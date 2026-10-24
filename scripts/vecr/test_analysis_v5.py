#!/usr/bin/env python3
"""Regression for cross-condition seed pseudoreplication in aggregate inference."""

import unittest
from analyze_agent_campaign_v5 import cluster_deltas, ci, wilson


class SeedClustering(unittest.TestCase):
    def test_repeated_conditions_do_not_inflate_independent_sample_count(self):
        rows = [
            dict(
                seed=str(seed),
                scenario=f"M{i:02d}",
                policy=policy,
                score=str(seed if policy == "P" else 0),
            )
            for seed in (0, 1)
            for i in range(19)
            for policy in ("B2", "P")
        ]
        clusters = cluster_deltas(rows, "score")
        self.assertEqual(clusters, [0.0, 1.0])
        mean, low, high = ci(clusters)
        self.assertEqual((mean, low, high), (0.5, 0.0, 1.0))

    def test_unseen_failures_and_missing_denominator_are_explicit(self):
        self.assertEqual(wilson(0, 0), (None, None))
        low, high = wilson(0, 100)
        self.assertEqual(low, 0)
        self.assertGreater(high, 0.03)
        self.assertLess(high, 0.04)
        self.assertEqual(wilson(100, 100)[1], 1)


if __name__ == "__main__":
    unittest.main()
