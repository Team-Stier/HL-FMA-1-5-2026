#!/usr/bin/env python3
"""Synthetic 13-section message replay; no ROS, physics, or vehicle output.

This runs the real MissionEngine, SelectorCore and SafetyGate. Coordinates,
landmarks, free-space approval and planner/perception replies are test fixtures.
Reverse requests remain blocked by the real unsigned vehicle-command contract;
the fixture continues supplying simulated poses so later missions can be checked.
For route projection and geometric safety integration see test/test_pipeline.py.
"""
import argparse
import json
import math
from pathlib import Path
import sys

PACKAGES = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(PACKAGES / name / 'src') for name in
                ('state_manager', 'selector', 'vehicle_safety')]
from selector.core import Candidate, Pose, SelectorCore, State, path_fingerprint
from stier_state_manager.mission import MissionEngine, PARKING_ROUTES
from vehicle_safety.core import Mission, PathReady, RawCommand, SafetyGate, SafetyInput


class SyntheticReplay:
    def __init__(self, parking_branch, lane):
        if parking_branch not in ('left', 'right') or lane not in ('left', 'right'):
            raise ValueError('Branch and lane must be left or right')
        self.parking_branch, self.lane = parking_branch, lane
        self.engine = MissionEngine({'rules': {'front_bumper_offset_m': .5}})
        self.selector, self.gate = SelectorCore(), SafetyGate()
        self.now, self.epoch, self.route = 1.0, 1, '1_right'
        self.request = ('1_right', 'RDDF', 1)
        self.events, self.last = [], None

    @property
    def section(self):
        return int(self.route.split('_')[0].split('-')[0])

    def step(self, s=0.0, speed=.5, signal='UNKNOWN', blocked=False,
             central=False, parking=False, lane=False, leg=None, path_available=True):
        self.now = round(self.now + .25, 8)
        section = self.section
        length = 18.0 if section in (5, 10) else 20.0
        marks = {'hill_start_s': 2.0, 'hill_stop_s': 5.0, 'hill_top_s': 9.0,
                 'stop_line_s': 10.0, 'intersection_exit_s': 15.0,
                 'parking_confirm_s': 18.0, 'parking_yaw_rad': math.pi,
                 'parking_exit_s': 19.0, 'lane_decision_s': 13.258, 'finish_s': 19.0}
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
        lanes = {'stamp': self.now, 'left': 'X', 'right': 'X'}
        lanes[self.lane] = 'DOWN'
        snapshot = {'now': self.now, 'decision_id': self.epoch,
                    'healthy': True, 'calibrated': True, 'route': self.route,
                    'section': section, 'length': length, 's': s, 'raw_s': s,
                    'at_end': s >= length, 'speed': speed, 'yaw': yaw,
                    'landmarks': {self.route: marks}, 'path_ready': selection.ready,
                    'signal': {'stamp': self.now, 'route': self.route, 'value': signal},
                    'dynamic': {'stamp': self.now, 'blocked': blocked, 'central_stopped': central},
                    'parking': spaces if parking else {}, 'lane': lanes if lane else {}}
        if leg is not None:
            snapshot['parking_maneuver'] = dict(leg, stamp=self.now,
                decision_id=self.epoch, route=self.route)
        decision = self.engine.update(snapshot)
        request = (self.route, decision['path_mode'], decision['direction'])
        if request != self.request or decision['reason'] == 'PARKING_LEG_ACCEPTED':
            self.request, self.epoch = request, self.epoch + 1
            decision.update(stop_requested=True, speed_limit=0.0, next_route=None)
            selection = self.selector.evaluate(State(self.now, self.now, self.epoch,
                                                     *self.request), candidates, self.now)
        valid = decision['phase'] not in ('FAULT', 'UNAVAILABLE')
        fingerprint = path_fingerprint(selection.candidate) if selection.ready else ''
        self.gate.update_mission(Mission(self.now, self.epoch, self.route, decision['path_mode'],
            decision['direction'], decision['speed_limit'], decision['stop_requested'], valid,
            'finish' in decision['completed_missions']), self.now)
        self.gate.update_safety(SafetyInput(self.now, blocked, True,
            'SYNTHETIC_OBSTACLE' if blocked else 'SYNTHETIC_FREE_SPACE', 0.0, fingerprint), self.now)
        self.gate.update_path(PathReady(self.now, self.epoch, self.route, decision['path_mode'],
            decision['direction'], selection.ready, fingerprint), self.now)
        self.gate.update_localization(True, self.now)
        self.gate.update_raw(RawCommand(10, 0, 0), self.now)
        command = self.gate.evaluate(self.now)
        event = {'time_s': self.now, 'route': self.route, 'section': section,
                 's_m': s, 'phase': decision['phase'], 'decision_id': self.epoch,
                 'direction': decision['direction'], 'stop_requested': decision['stop_requested'],
                 'reason': decision['reason'], 'selector_ready': selection.ready,
                 'gate_allowed': command.allowed, 'gate_reason': command.reason,
                 'selected_branch': decision['selected_branch'], 'next_route': decision['next_route']}
        self.events.append(event)
        self.last = decision
        return event

    def advance(self, expected):
        if self.last.get('next_route') != expected:
            raise AssertionError('Expected transition to {}, received {}'.format(expected, self.events[-1]))
        self.route = expected
        mode = 'LOCAL' if self.section == 3 else 'PARKING' if self.section in (5, 6, 10, 11) else 'RDDF'
        self.request = (expected, mode, 1)
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

    @staticmethod
    def leg(phase, index, start, target, final):
        return {'phase': phase, 'leg_index': index, 'start_s': float(start),
                'target_s': float(target), 'final_leg': final,
                'direction': -1 if phase == 'REVERSE_ENTRY' else 1}

    def park(self, kind):
        entry, exit_route = PARKING_ROUTES[kind][self.parking_branch]
        self.advance(entry)
        if kind == 't':
            approach = self.leg('FORWARD_APPROACH', 0, 0, 5, False)
            self.step(speed=0.0, leg=approach)
            self.step(s=2.0, leg=approach)
            self.step(s=5.0, speed=0.0, leg=approach)
            reverse = self.leg('REVERSE_ENTRY', 1, 5, 18, True)
            self.step(s=5.0, speed=0.0, leg=reverse)
        else:
            reverse = self.leg('REVERSE_ENTRY', 0, 0, 18, True)
            self.step(speed=0.0, leg=reverse)
        reversing = self.step(s=12.0, speed=-.5, leg=reverse)
        if reversing['gate_reason'] != 'REVERSE_INTERFACE_UNAVAILABLE':
            raise AssertionError('Unsigned command gate must reject synthetic reverse request')
        for _ in range(4):
            self.step(s=18.0, speed=0.0, leg=reverse)
        self.advance(exit_route)
        exit_leg = self.leg('FORWARD_EXIT', 0, 0, 19, True)
        self.step(speed=0.0, leg=exit_leg)
        self.step(s=10.0, leg=exit_leg)
        self.step(s=20.0, leg=exit_leg)

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
        self.step(s=5.0, blocked=True)
        for _ in range(14):
            self.step(s=5.0, speed=0.0, blocked=True, central=True)
        self.step(s=5.0)
        self.step(s=20.0)
        self.advance('9')
        for s in (0.0, 1.0, 2.0):
            self.step(s=s, parking=True)
        self.step(s=20.0, parking=True)
        self.park('parallel')
        self.advance('12')
        self.step(s=13.258, speed=0.0)
        for _ in range(3):
            self.step(s=13.258, speed=0.0, lane=True)
        if self.lane == 'right':
            self.step(s=20.0, lane=True)
        self.advance('13_' + self.lane)
        self.step()
        self.step(s=19.0)
        if 'finish' not in self.last['completed_missions']:
            raise AssertionError('Synthetic mission replay did not finish')
        return {'simulation_only': True, 'fixture': 'SYNTHETIC_LOGICAL_MESSAGES_NOT_DRIVING_SIMULATION',
                'parking_branch': self.parking_branch, 'lane': self.lane,
                'vehicle_output': False, 'reverse_output_supported': False,
                'completed_missions': self.last['completed_missions'],
                'sections': sorted({event['section'] for event in self.events}), 'events': self.events}


def replay(parking_branch='left', lane='left'):
    return SyntheticReplay(parking_branch, lane).run()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parking-branch', choices=('left', 'right'), default='left')
    parser.add_argument('--lane', choices=('left', 'right'), default='left')
    parser.add_argument('--output', type=Path, help='Optional JSON trace destination')
    args = parser.parse_args()
    result = replay(args.parking_branch, args.lane)
    if args.output:
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print('SYNTHETIC message replay only; no ROS or vehicle output. Reverse gate remains blocked.')
    print('Sections: ' + ', '.join(map(str, result['sections'])))
    for event in result['events']:
        print('{time_s:6.2f} {route:23} {phase:23} request={decision_id:<3} '
              'dir={direction:2} gate={gate_reason}'.format(**event))
    print('Completed missions: ' + ', '.join(sorted(result['completed_missions'])))


if __name__ == '__main__':
    main()
