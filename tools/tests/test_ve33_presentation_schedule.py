"""本地抓握的画面调度不应降低物理采样频率。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ve33_g4_view import GraspPresentationSchedule


class PresentationScheduleTest(unittest.TestCase):
    def test_render_is_throttled_without_skipping_physics(self):
        schedule = GraspPresentationSchedule()
        physics = []
        presentations = []
        for tick in range(11):
            now = tick * .01
            physics.append(now)
            if schedule.due(now, 1):
                presentations.append(round(now, 3))
        self.assertEqual(len(physics), 11)
        self.assertEqual(presentations, [0., .04, .08])

    def test_scene_revision_forces_new_presentation(self):
        schedule = GraspPresentationSchedule()
        self.assertTrue(schedule.due(1., 7))
        self.assertFalse(schedule.due(1.01, 7))
        self.assertTrue(schedule.due(1.02, 8))
        self.assertFalse(schedule.due(1.04, 8))
        self.assertTrue(schedule.due(1.06, 8))


if __name__ == '__main__':
    unittest.main()
