"""localization_v2's vocabulary, ROS-free, in one place.

The node publishes ~/status as a Float32MultiArray in STATUS_FIELDS order;
log_localization writes it as v2_* columns; tools/replay_localization_v2.py
reproduces the same fields offline; tools/segment_track.py writes MODES into
segments.csv. One definition, four readers.
"""

MODES = ('STRAIGHT_BLIND', 'APPROACH', 'CORNER', 'TRANSIT')
MODE_ID = {m: i for i, m in enumerate(MODES)}

STATUS_FIELDS = (
    'mode',          # MODES index in force for this scan
    'along_info',    # live information along the heading (the guard reads this)
    'cross_info',    # ... and across it
    'dx_meas',       # the matcher's suggested correction, map frame
    'dy_meas',
    'dx_appl',       # what this scan actually moved the PUBLISHED correction
    'dy_appl',
    'inlier_frac',
    'resid_m',       # mean inlier endpoint-to-wall distance; -1 when undefined
    'yaw_hint_deg',  # the yaw a 3-DOF solve wanted; the invariant says ~0
    'iters',
    'compute_ms',
    'reject',        # REJECT_CODES index; 0 = accepted
    'clamp_m',       # metres of innovation the per-axis plausibility clamp removed
    'pending_m',     # correction still owed by the rate limiter
    'k_scale',       # estimated odometry scale error (0.02 = DR over-reads 2 %)
    'sig_along',     # posterior 1-sigma along the heading [m]
    'sig_cross',
)

# 'guard' is not a rejection of the scan: the along component was dropped by
# the live observability guard and the cross component was fused.
REJECT_CODES = ('ok', 'matcher', 'clearance', 'no_odom', 'not_seeded', 'guard')
