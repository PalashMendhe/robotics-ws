#!/usr/bin/env python3
"""
Tests for broadcaster_node.py's monotonic-stamp policy — no simulator needed.

The broadcaster converts Gazebo odometry into the odom -> base_footprint TF
transform. AMCL's motion model goes inconsistent (TF_OLD_DATA warnings, and
in the worst case localization jumps) if TF ever receives a stale or
duplicated timestamp, so the broadcaster enforces strictly-monotonic stamps
and drops the zero stamp emitted before the Gazebo clock is published.

This policy was extracted into the pure function should_accept_stamp()
precisely so it can be verified here: a regression that lets a single
out-of-order stamp through only reproduces in a full sim run, but fails
this suite in milliseconds.
"""


from nav_nodes.broadcaster_node import should_accept_stamp


class TestShouldAcceptStamp:
    def test_first_stamp_accepted(self):
        assert should_accept_stamp(1_000, 0) is True

    def test_zero_stamp_dropped(self):
        """Zero means Gazebo is running but the clock is not synced yet."""
        assert should_accept_stamp(0, 0) is False
        assert should_accept_stamp(0, 5_000) is False

    def test_duplicate_stamp_dropped(self):
        assert should_accept_stamp(1_000, 1_000) is False

    def test_out_of_order_stamp_dropped(self):
        assert should_accept_stamp(999, 1_000) is False

    def test_strictly_increasing_sequence_fully_accepted(self):
        last = 0
        stamps = [100, 200, 300, 400, 500]
        for stamp in stamps:
            assert should_accept_stamp(stamp, last) is True
            last = stamp

    def test_realistic_stream_with_duplicates_and_reorder(self):
        """A mixed stream: only the fresh stamps get through."""
        stream = [0, 10, 10, 9, 12, 11, 13, 0, 14]
        accepted = []
        last = 0
        for stamp in stream:
            if should_accept_stamp(stamp, last):
                accepted.append(stamp)
                last = stamp
        assert accepted == [10, 12, 13, 14]

    def test_no_regression_to_equal_stamps(self):
        """
        Equal stamps are the common case and must never reach TF.

        Odom is published at a fixed rate while the sim clock stalls.
        """
        assert should_accept_stamp(42, 42) is False
