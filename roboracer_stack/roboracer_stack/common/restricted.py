"""The competition's restricted-topic list, as code rather than as a comment.

Legal inputs during racing: LiDAR, camera, IMU, wheel encoders, and
steering/throttle feedback. Everything below is simulator ground truth or race
telemetry and must not reach the control path.

WHY THIS EXISTS
---------------
The boundary used to be documentation: 33 comments across nine files saying
RESTRICTED, and not one runtime check. pure_pursuit's `pose_topic` DEFAULTED to
the ground-truth odometry topic, so several development runs drove on a perfect
pose and produced lap times that looked race-legal. Nothing printed a word.

A comment cannot fail. A log line can. Any node that subscribes to one of these
calls `warn(self, topic, why)` and the terminal says so, once, in red.

THE WARMUP WINDOW
-----------------
"Restricted" is about WHEN and HOW OFTEN, not only about which topic. The first
lap is a warmup and the timer starts after it, so these topics are readable up
to that point. What is not acceptable is a node that keeps reading ground truth
into the timed laps.

So there are two access patterns, and they get two functions:

    seed(node, topic, why)   ONE read, before the car has moved, after which
                             the subscription is destroyed. Legal: it happens
                             inside the warmup window and never repeats. This
                             is how localization gets its initial pose.

    warn(node, topic, why)   A CONTINUOUS subscription. Not legal for a timed
                             run, whatever the topic; logged in red.

The distinction is load-bearing. Seeding from /ips once is the difference
between slam_toolbox starting on the true pose and starting on a hardcoded
guess -- and slam_toolbox has no global relocalization to recover a wrong guess,
so without the seed a wrong constant is frozen for the whole run. Treating that
single read as equivalent to streaming ground truth into the control path is
what made the seed look illegal and pushed race runs onto the guess instead.

A node that seeds MUST then call released(), so the terminal shows ground truth
actually being let go rather than merely promised.
"""

from roboracer_stack.common.frames import NS

# topic -> what makes it restricted
RESTRICTED = {
    f'{NS}/odom': 'simulator ground-truth pose and twist',
    f'{NS}/ips': 'simulator ground-truth position',
    f'{NS}/lap_count': 'race telemetry',
    f'{NS}/lap_time': 'race telemetry',
    f'{NS}/last_lap_time': 'race telemetry',
    f'{NS}/best_lap_time': 'race telemetry',
    f'{NS}/collision_count': 'race telemetry',
    '/tf_ground_truth': "the devkit's ground-truth TF, remapped off /tf",
}


def is_restricted(topic):
    """True if `topic` may not be read during a competition run."""
    return topic in RESTRICTED


def warn(node, topic, why=''):
    """Log loudly if `topic` is CONTINUOUSLY subscribed. Returns True if it was.

    Call once per subscription, at construction. For a one-shot read inside the
    warmup window use seed() instead -- that one is legal, and logging it in red
    trains you to ignore the red. Deliberately does NOT raise: development runs
    read these on purpose, and a node that refuses to start is worse than one
    that says what it is doing.
    """
    if topic not in RESTRICTED:
        return False
    tail = f' -- {why}' if why else ''
    node.get_logger().error(
        f'RESTRICTED TOPIC: reading {topic} ({RESTRICTED[topic]}){tail}. '
        'This run is NOT race-legal; any lap time from it is a development '
        'number. See common/restricted.py.')
    return True


def seed(node, topic, why=''):
    """Announce a ONE-SHOT read of a restricted topic in the warmup window.

    Legal, unlike warn(): the first lap is warmup and the timer starts after it,
    so a single read before the car moves sits inside the permitted window. The
    caller is promising two things -- that it reads once, and that it destroys
    the subscription afterwards and says so with released(). Returns True if the
    topic was a restricted one.
    """
    if topic not in RESTRICTED:
        return False
    tail = f' -- {why}' if why else ''
    node.get_logger().info(
        f'WARMUP SEED: reading {topic} ({RESTRICTED[topic]}) ONCE{tail}. Legal: '
        'the first lap is warmup, this is read before the car moves, and it is '
        'released immediately after. Nothing reads ground truth afterwards.')
    return True


def released(node, topics):
    """Confirm that one-shot ground-truth subscriptions are now gone.

    Call right after destroying them. The point is that the terminal shows the
    release happening, so "it only reads it once" is observable rather than a
    claim in a docstring.
    """
    gone = [t for t in topics if t in RESTRICTED]
    if gone:
        node.get_logger().info(
            f'{node.get_name()}: released {", ".join(gone)} -- no restricted '
            'topic is read from here on; the timed laps are clean.')
    return gone


def banner(node, topics):
    """Summarise a node's restricted inputs at startup, or confirm there are none.

    `topics` is whatever the node actually subscribed to.
    """
    bad = [t for t in topics if t in RESTRICTED]
    if bad:
        node.get_logger().error(
            f'{node.get_name()}: DEVELOPMENT MODE -- {len(bad)} restricted '
            f'input(s): {", ".join(bad)}')
    else:
        node.get_logger().info(f'{node.get_name()}: race-legal inputs only')
    return bad
