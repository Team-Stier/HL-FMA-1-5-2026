#!/usr/bin/env python3
"""Synthetic 13-section message replay; no ROS, physics, or vehicle output.

This runs the real MissionEngine and SelectorCore. Coordinates,
landmarks, free-space approval and planner/perception replies are test fixtures.
No ROS node or physical vehicle command is started.
"""
import argparse
import json
import math
from pathlib import Path
import sys

PACKAGES = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(PACKAGES / name / 'src') for name in
                ('state_manager', 'selector')]
from selector.core import Candidate, Pose, SelectorCore, State
from stier_state_manager.mission import MissionEngine, PARKING_ROUTES


class SyntheticReplay:
    def __init__(self, parking_branch, finish_branch):
        if parking_branch not in ('left', 'right') or finish_branch not in ('left', 'right'):
            raise ValueError('Parking and finish branches must be left or right')
        self.parking_branch, self.finish_branch = parking_branch, finish_branch
        self.engine = MissionEngine({'rules': {'front_bumper_offset_m': .5},
                                     'finish_fallback_branch': finish_branch,
                                     'parking_branches': {
                                         't': parking_branch,
                                         'parallel': parking_branch,
                                     }})
        self.selector = SelectorCore()
        self.now, self.epoch, self.route = 1.0, 1, '1_right'
        self.request = ('1_right', 'RDDF', 1)
        self.events, self.last = [], None

    @property
    def section(self):
        return int(self.route.split('_')[0].split('-')[0])

    def step(self, s=0.0, speed=.5, signal='UNKNOWN', parking=False,
             path_available=True):
        self.now = round(self.now + .25, 8)
        section = self.section
        length = 18.0 if section in (5, 10) else 23.0 if section == 13 else 20.0
        marks = {'hill_start_s': 2.0, 'hill_stop_s': 5.0, 'hill_top_s': 9.0,
                 'stop_line_s': 10.0,
                 'parking_confirm_s': 18.0, 'parking_yaw_rad': math.pi,
                 'parking_exit_s': 19.0}
        name, mode, direction = self.request
        yaw = math.pi if direction < 0 else 0.0
        poses = tuple(Pose('map', (float(x), 0.0, 0.0),
                      (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
                      for x in range(21))
        candidate = Candidate(self.now, self.now, self.epoch, name, direction,
                              'map', 'map', self.now, poses)
        candidates = {mode: candidate} if path_available else {}
        selection = self.selector.evaluate(State(self.now, self.now, self.epoch,
                                                 *self.request), candidates, self.now)
        spaces = {'stamp': self.now, 'left': 'BLOCKED', 'right': 'BLOCKED'}
        spaces[self.parking_branch] = 'CLEAR'
        lanes = {'stamp': self.now, 'route': self.route, 'left': 'X', 'right': 'X'}
        lanes[self.finish_branch] = 'DOWN'
        snapshot = {'now': self.now, 'decision_id': self.epoch,
                    'healthy': True, 'calibrated': True, 'route': self.route,
                    'section': section, 'length': length, 's': s, 'raw_s': s,
                    'at_end': s >= length, 'speed': speed, 'yaw': yaw,
                    'finish_branch_s': 13.258,
                    'landmarks': {self.route: marks}, 'path_ready': selection.ready,
                    'signal': {'stamp': self.now, 'route': self.route, 'value': signal},
                    'lane': lanes, 'parking': spaces if parking else {}}
        decision = self.engine.update(snapshot)
        request = (self.route, decision['path_mode'], decision['direction'])
        if request != self.request:
            self.request, self.epoch = request, self.epoch + 1
            decision.update(stop_requested=True, speed_limit=0.0, next_route=None)
            selection = self.selector.evaluate(State(self.now, self.now, self.epoch,
                                                     *self.request), candidates, self.now)
        valid = decision['phase'] not in ('FAULT', 'UNAVAILABLE')
        control_allowed = (selection.ready and valid and
                           not decision['stop_requested'] and
                           decision['speed_limit'] > 0.0)
        control_reason = ('ACTIVE' if control_allowed else
                          decision['reason'] if decision['stop_requested'] else
                          'PATH_NOT_READY' if not selection.ready else 'MISSION_INVALID')
        event = {'time_s': self.now, 'route': self.route, 'section': section,
                 's_m': s, 'phase': decision['phase'], 'decision_id': self.epoch,
                 'direction': decision['direction'], 'stop_requested': decision['stop_requested'],
                 'reason': decision['reason'], 'selector_ready': selection.ready,
                 'control_allowed': control_allowed, 'control_reason': control_reason,
                 'selected_branch': decision['selected_branch'], 'next_route': decision['next_route']}
        self.events.append(event)
        self.last = decision
        return event

    def advance(self, expected):
        if self.last.get('next_route') != expected:
            raise AssertionError('Expected transition to {}, received {}'.format(expected, self.events[-1]))
        self.route = expected
        mode = 'LOCAL' if self.section == 3 else 'RDDF'
        self.request = (expected, mode, -1 if self.section in (5, 11) else 1)
        self.epoch += 1

    def traffic(self, parking=False, left=False):
        self.step(s=9.5, speed=0.0, signal='RED', parking=parking)
        if left:
            rejected = self.step(s=9.5, speed=0.0, signal='GREEN')
            if not rejected['stop_requested']:
                raise AssertionError('General green incorrectly authorized protected left turn')
        self.step(s=9.5, signal='LEFT_ARROW' if left else 'GREEN', parking=parking)
        self.step(s=16.0, signal='RED', parking=parking)
        self.step(s=20.0, signal='RED', parking=parking)

    def park(self, kind):
        entry, exit_route = PARKING_ROUTES[kind][self.parking_branch]
        self.advance(entry)
        if kind == 't':
            reversing = self.step(s=12.0, speed=-.5)
            if not reversing['control_allowed']:
                raise AssertionError('T-parking reverse RDDF should be commandable')
            self.step(s=18.0, speed=0.0)
            self.advance(exit_route)
            for _ in range(9):
                self.step(speed=0.0)
            self.step(s=10.0)
            self.step(s=20.0)
            return
        if self.parking_branch == 'left':
            entry_changes = (9.337439695228316,)
        else:
            entry_changes = (6.614062696750327, 16.56201644939806)
        for position in entry_changes:
            self.step(s=position - .1, speed=0.0)
            self.step(s=position + .1)
        self.step(s=18.0, speed=0.0)
        self.advance(exit_route)
        exit_change = (.7236489020465036 if self.parking_branch == 'left'
                       else 2.237988918723955)
        self.step(s=exit_change - .1, speed=0.0)
        self.step(s=exit_change + .1)
        self.step(s=20.0)

    def run(self):
        self.step()
        for _ in range(14):
            self.step(s=5.0, speed=0.0)
        self.step(s=9.0)
        self.step(s=20.0)
        self.advance('2')
        self.step()
        self.traffic()
        self.advance('3_s-static-obstacle')
        self.step(path_available=False)
        self.step()
        self.step(s=20.0)
        self.advance('4')
        for s in (0.0, 1.0, 2.0):
            self.step(s=s, parking=True)
        self.traffic(parking=True)
        self.park('t')
        self.advance('7')
        self.step()
        self.traffic(left=True)
        self.advance('8_dynamic-obstacle')
        self.step()
        self.step(s=20.0)
        self.advance('9')
        for s in (0.0, 1.0, 2.0):
            self.step(s=s, parking=True)
        self.step(s=20.0, parking=True)
        self.park('parallel')
        self.advance('12')
        for _ in range(3):
            self.step(s=12.0, speed=0.0)
        self.step(s=13.258, speed=0.0)
        if self.finish_branch == 'right':
            self.step(s=20.0)
        self.advance('13_' + self.finish_branch)
        self.step()
        self.step(s=23.0)
        if 'finish' not in self.last['completed_missions']:
            raise AssertionError('Synthetic mission replay did not finish')
        return {'simulation_only': True, 'fixture': 'SYNTHETIC_LOGICAL_MESSAGES_NOT_DRIVING_SIMULATION',
                'parking_branch': self.parking_branch, 'finish_branch': self.finish_branch,
                'vehicle_output': False, 'reverse_output_supported': True,
                'completed_missions': self.last['completed_missions'],
                'sections': sorted({event['section'] for event in self.events}), 'events': self.events}


def replay(parking_branch='left', finish_branch='left'):
    return SyntheticReplay(parking_branch, finish_branch).run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parking-branch', choices=('left', 'right'), default='left')
    parser.add_argument('--finish-branch', choices=('left', 'right'), default='left')
    parser.add_argument('--output', type=Path, help='Optional JSON trace destination')
    args = parser.parse_args()
    result = replay(args.parking_branch, args.finish_branch)
    if args.output:
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('SYNTHETIC message replay only; no ROS or vehicle output.')
    print('Sections: ' + ', '.join(map(str, result['sections'])))
    for event in result['events']:
        print('{time_s:6.2f} {route:23} {phase:23} request={decision_id:<3} '
              'dir={direction:2} control={control_reason}'.format(**event))
    print('Completed missions: ' + ', '.join(sorted(result['completed_missions'])))


if __name__ == '__main__':
    main()
