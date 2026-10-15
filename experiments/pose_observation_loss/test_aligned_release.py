"""Independent geometry and admission-boundary checks for aligned release."""

import unittest

from verify_stopping_preview import exact_release_preview, preview_components
from test_recovery_preview_geometry import inputs


class AlignedReleaseTests(unittest.TestCase):
    def test_clear_stop_cannot_release_with_blocked_terminal_entry(self):
        before, row = inputs()
        row.update(terminal_entry_only="true", aligned_terminal_release="true")
        stop, terminal, _ = preview_components(before, row, [(26.18, 25)])
        self.assertGreater(stop, 0.55)
        self.assertLess(terminal - 0.1, 0.5)
        self.assertLess(exact_release_preview(stop, terminal, row), 0.55)

    def test_terminal_entry_boundary_is_strict_with_stop_hysteresis_retained(self):
        row = dict(aligned_terminal_release="true")
        self.assertLessEqual(exact_release_preview(1, 0.6, row), 0.55)
        self.assertGreater(exact_release_preview(1, 0.600001, row), 0.55)
        self.assertLessEqual(exact_release_preview(0.55, 1, row), 0.55)

    def test_release_between_original_combined_and_stop_only_rules(self):
        aligned = dict(aligned_terminal_release="true")
        combined = dict(recovery_preview_active="true")
        stop_only = dict(recovery_preview_active="true", terminal_entry_only="true")
        for stop in [0.5, 0.6, 0.8, 1]:
            for terminal in [0.5, 0.6, 0.8, 1]:
                original = exact_release_preview(stop, terminal, combined)
                candidate = exact_release_preview(stop, terminal, aligned)
                loose = exact_release_preview(stop, terminal, stop_only)
                self.assertLessEqual(original, candidate)
                self.assertLessEqual(candidate, loose)
        self.assertEqual(exact_release_preview(1, 0, {}), 1)


if __name__ == "__main__":
    unittest.main()
