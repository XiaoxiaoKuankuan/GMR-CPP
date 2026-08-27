#include "gmr/foot_contact.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

gmr::FootObservation observation(gmr::FootSide side, double z,
                                 double x = 0.0) {
    gmr::FootObservation value;
    value.side = side;
    value.points_world = {
        Eigen::Vector3d(x - 0.05, -0.03, z),
        Eigen::Vector3d(x - 0.05, 0.03, z),
        Eigen::Vector3d(x + 0.12, -0.03, z),
        Eigen::Vector3d(x + 0.12, 0.03, z),
    };
    for (const auto& point : value.points_world) value.center_world += point;
    value.center_world /= 4.0;
    return value;
}

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    try {
        gmr::FootContactDetectorConfig config;
        config.allow_flight = true;
        config.enter_frames = 3;
        config.exit_frames = 2;
        config.minimum_stance_frames = 0;
        gmr::FootContactDetector detector(config);

        gmr::FootContactState state;
        for (int frame = 0; frame < 3; ++frame) {
            state = detector.update(
                observation(gmr::FootSide::Left, 0.01),
                observation(gmr::FootSide::Right, 0.012), frame * 0.02);
        }
        require(state.left_stance && state.right_stance,
                "stable low feet should enter double support");

        state = detector.update(
            observation(gmr::FootSide::Left, 0.06),
            observation(gmr::FootSide::Right, 0.012), 0.06);
        require(state.left_stance, "exit hysteresis must retain left foot for one frame");
        state = detector.update(
            observation(gmr::FootSide::Left, 0.06),
            observation(gmr::FootSide::Right, 0.012), 0.08);
        require(!state.left_stance && state.right_stance,
                "lifted left foot should leave support while right remains");

        // Noise between the enter and exit thresholds must not chatter.
        for (int frame = 0; frame < 8; ++frame) {
            const double noise_z = frame % 2 == 0 ? 0.030 : 0.034;
            state = detector.update(
                observation(gmr::FootSide::Left, 0.06),
                observation(gmr::FootSide::Right, noise_z), 0.10 + frame * 0.02);
            require(state.right_stance, "contact hysteresis chattered in threshold band");
        }

        // A foot can remain close to the ground while sweeping through a turn.
        // Horizontal velocity must release it instead of pinning it as stance.
        gmr::FootContactDetectorConfig moving_config;
        moving_config.allow_flight = true;
        moving_config.enter_frames = 1;
        moving_config.exit_frames = 1;
        moving_config.minimum_stance_frames = 0;
        gmr::FootContactDetector moving_detector(moving_config);
        state = moving_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.0),
            observation(gmr::FootSide::Right, 0.08, 0.0), 2.0);
        require(state.left_stance, "low stationary foot should enter stance");
        state = moving_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.01),
            observation(gmr::FootSide::Right, 0.08, 0.0), 2.02);
        require(!state.left_stance,
                "fast horizontal foot motion must release stance immediately");

        // Motion below the latch exit speed can still make two simultaneous
        // world anchors undesirable.  Dynamic double support keeps only the
        // stationary primary foot.
        gmr::FootContactDetectorConfig primary_config;
        primary_config.allow_flight = true;
        primary_config.enter_frames = 1;
        primary_config.exit_frames = 1;
        primary_config.minimum_stance_frames = 0;
        gmr::FootContactDetector primary_detector(primary_config);
        state = primary_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.0),
            observation(gmr::FootSide::Right, 0.01, 0.0), 3.0);
        require(state.left_stance && state.right_stance,
                "stationary double support should keep both anchors");
        state = primary_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.001),
            observation(gmr::FootSide::Right, 0.01, 0.0), 3.02);
        require(!state.left_stance && state.right_stance,
                "dynamic double support did not select a primary foot");

        // Ambiguous moving double support must keep one primary side instead
        // of alternating its forced support every frame.
        gmr::FootContactDetectorConfig sticky_config;
        sticky_config.allow_flight = true;
        sticky_config.enter_frames = 1;
        sticky_config.exit_frames = 1;
        sticky_config.minimum_stance_frames = 0;
        sticky_config.max_enter_horizontal_speed = 0.15;
        sticky_config.max_exit_horizontal_speed = 0.20;
        sticky_config.double_support_max_horizontal_speed = 0.025;
        gmr::FootContactDetector sticky_detector(sticky_config);
        sticky_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.0),
            observation(gmr::FootSide::Right, 0.01, 0.0), 4.0);
        state = sticky_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.001),
            observation(gmr::FootSide::Right, 0.01, 0.002), 4.02);
        require(state.left_stance && state.left_forced && !state.right_stance,
                "ambiguous support did not initially select the slower left foot");
        state = sticky_detector.update(
            observation(gmr::FootSide::Left, 0.01, 0.003),
            observation(gmr::FootSide::Right, 0.01, 0.003), 4.04);
        require(state.left_stance && state.left_forced && !state.right_stance,
                "ambiguous support side chattered when foot speeds crossed");

        gmr::FootContactDetectorConfig no_flight = config;
        no_flight.allow_flight = false;
        gmr::FootContactDetector grounded(no_flight);
        state = grounded.update(
            observation(gmr::FootSide::Left, 0.08),
            observation(gmr::FootSide::Right, 0.06), 1.0);
        require(!state.left_stance && state.right_stance && state.right_forced,
                "no-flight fallback must choose the lower foot");
        state = grounded.update(
            observation(gmr::FootSide::Left, 0.01),
            observation(gmr::FootSide::Right, 0.06), 1.02);
        require(state.right_forced,
                "forced support must remain sticky while detected contact is pending");
        for (int frame = 0; frame < 3; ++frame)
            state = grounded.update(
                observation(gmr::FootSide::Left, 0.01),
                observation(gmr::FootSide::Right, 0.06), 1.04 + frame * 0.02);
        require(state.left_stance && !state.left_forced,
                "detected contact must replace forced fallback");

        gmr::SourceGroundTrackerConfig tracker_config;
        tracker_config.enabled = true;
        tracker_config.support_track_speed = 0.10;
        tracker_config.reacquire_track_speed = 0.20;
        tracker_config.max_reacquire_vertical_speed = 1.0;
        tracker_config.max_reacquire_horizontal_speed = 1.0;
        tracker_config.reacquire_after_frames = 3;
        tracker_config.stable_reacquire_frames = 2;
        gmr::SourceGroundTracker tracker(tracker_config);
        gmr::FootContactState support;
        support.left_stance = true;
        support.right_stance = true;
        tracker.update(
            observation(gmr::FootSide::Left, 0.0),
            observation(gmr::FootSide::Right, 0.0), support, 5.0);
        tracker.update(
            observation(gmr::FootSide::Left, 0.02),
            observation(gmr::FootSide::Right, 0.02), support, 5.1);
        require(std::abs(tracker.groundZ() - 0.01) < 1e-9,
                "stance ground tracking exceeded its causal speed cap");

        gmr::FootContactState flight;
        tracker.update(
            observation(gmr::FootSide::Left, 0.10),
            observation(gmr::FootSide::Right, 0.11), flight, 5.2);
        tracker.update(
            observation(gmr::FootSide::Left, 0.10),
            observation(gmr::FootSide::Right, 0.11), flight, 5.3);
        require(std::abs(tracker.groundZ() - 0.01) < 1e-9 &&
                    !tracker.reacquiring(),
                "ordinary flight must freeze the source ground");
        tracker.update(
            observation(gmr::FootSide::Left, 0.10),
            observation(gmr::FootSide::Right, 0.11), flight, 5.4);
        require(tracker.reacquiring() &&
                    std::abs(tracker.groundZ() - 0.03) < 1e-9,
                "long stable flight did not gradually reacquire drifted ground");

        std::cout << "foot_contact_detector_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "foot_contact_detector_test: FAIL: " << error.what() << '\n';
        return 1;
    }
}
