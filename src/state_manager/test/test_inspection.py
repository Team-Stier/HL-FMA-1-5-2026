"""An observed segment must remain distinct from mission authorization."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from stier_state_manager.geometry import Route
from stier_state_manager.inspection import inspection_specs


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.routes = {'7': Route('7', [(0, 0, 0), (10, 0, 0)])}
        self.samples = {
            'observed': dict(stamp=10., received=10., frame='map', matched=True,
                             has_nearest=True, source='7', reason='MATCHED'),
            'localization_valid': dict(stamp=10., received=10., value=True),
            'mission': dict(stamp=10., received=10., text='S01 | CALIBRATION_REQUIRED'),
            'local': dict(stamp=10., received=10., frame='map', route='7', decision_id=2,
                          points=[(0., 0., .3), (2., 0., .3)])}

    def specs(self, now=10.1):
        return inspection_specs(self.routes, self.samples, now)

    def test_observed_route_is_independent_of_stopped_mission(self):
        specs = self.specs()
        text = specs[-1]['text']
        self.assertIn('Observed S07: 7', text)
        self.assertIn('Manager: S01 | CALIBRATION_REQUIRED', text)
        self.assertIn('Traffic signal: NO_DATA', text)
        self.assertIn('no motion authorization', text)
        self.assertIn('candidate_local', {m['ns'] for m in specs})

    def test_stale_data_removes_observed_route_and_candidate(self):
        specs = self.specs(11.)
        self.assertEqual([m['ns'] for m in specs], ['inspection_status'])
        self.assertIn('STALE', specs[-1]['text'])

    def test_future_data_cannot_highlight_route(self):
        self.assertEqual(len(self.specs(9.)), 1)

    def test_invalid_localization_removes_observed_highlight(self):
        self.samples['localization_valid']['value'] = False
        self.assertNotIn('observed_rddf', {m['ns'] for m in self.specs()})

    def test_ambiguous_match_nearest_is_not_an_observed_route(self):
        self.samples['observed'].update(matched=False, reason='AMBIGUOUS_ROUTE')
        self.assertNotIn('observed_rddf', {m['ns'] for m in self.specs()})
        self.assertIn('AMBIGUOUS_ROUTE', self.specs()[-1]['text'])

    def test_wrong_frame_candidate_is_not_drawn(self):
        self.samples['local']['frame'] = 'odom'
        self.assertNotIn('candidate_local', {m['ns'] for m in self.specs()})
        self.assertIn('FRAME_MISMATCH', self.specs()[-1]['text'])

    def test_empty_roi_does_not_claim_free_space(self):
        self.samples['roi'] = dict(stamp=10., received=10., text='0 points (not a free-space verdict)')
        self.assertIn('not a free-space verdict', self.specs()[-1]['text'])

    def test_nonfinite_points_are_not_drawn(self):
        self.samples['local']['points'][0] = (float('nan'), 0., 0.)
        self.assertNotIn('candidate_local', {m['ns'] for m in self.specs()})


if __name__ == '__main__':
    unittest.main()
