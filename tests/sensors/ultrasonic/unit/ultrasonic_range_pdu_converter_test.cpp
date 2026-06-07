#include "hakoniwa/pdu/converter/sensor_msgs/range.hpp"
#include "sensors/ultrasonic/ultrasonic_sensor.hpp"
#include "tests/sensors/support/sensor_test_utils.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace
{
using hako::robots::sensor::test::NearlyEqual;
using hako::robots::sensor::ultrasonic::IUltrasonicSensor;
using hako::robots::sensor::ultrasonic::RangeRadiationType;
using hako::robots::sensor::ultrasonic::UltrasonicConfig;
using hako::robots::sensor::ultrasonic::UltrasonicFrame;
using hako::robots::sensor::ultrasonic::UltrasonicStatus;

void TestRangePduConversion()
{
    UltrasonicConfig config {};
    config.frame_id = "front_ultrasonic";
    config.radiation_type = RangeRadiationType::ULTRASOUND;
    config.cone.horizontal = 1.221730476;
    config.detection_distance.min = 0.05;
    config.detection_distance.max = 2.0;

    UltrasonicFrame frame {};
    frame.frame_id = "front_ultrasonic";
    frame.range = 0.86;
    frame.variance = 0.25;
    frame.status = UltrasonicStatus::OK;

    const HakoCpp_Range pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(config, frame);

    HAKO_TEST_EXPECT(pdu.header.frame_id == "front_ultrasonic", "unexpected frame_id");
    HAKO_TEST_EXPECT(pdu.radiation_type == 0, "unexpected radiation type");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.field_of_view, static_cast<float>(config.cone.horizontal)), "unexpected field_of_view");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.min_range, 0.05F), "unexpected min_range");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.max_range, 2.0F), "unexpected max_range");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.range, 0.86F), "unexpected range");
}

void TestInfraredRadiationTypeConversion()
{
    UltrasonicConfig config {};
    config.frame_id = "ir_range";
    config.radiation_type = RangeRadiationType::INFRARED;
    config.cone.horizontal = 0.5;
    config.detection_distance.min = 0.1;
    config.detection_distance.max = 1.5;

    UltrasonicFrame frame {};
    frame.range = 1.25;

    const HakoCpp_Range pdu = hako::robots::pdu::converter::sensor_msgs::ToHakoPdu(config, frame);

    HAKO_TEST_EXPECT(pdu.header.frame_id == "ir_range", "unexpected infrared frame_id");
    HAKO_TEST_EXPECT(pdu.radiation_type == 1, "unexpected infrared radiation type");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.field_of_view, 0.5F), "unexpected infrared field_of_view");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.min_range, 0.1F), "unexpected infrared min_range");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.max_range, 1.5F), "unexpected infrared max_range");
    HAKO_TEST_EXPECT(NearlyEqual(pdu.range, 1.25F), "unexpected infrared range");
}
}

int main()
{
    try {
        TestRangePduConversion();
        TestInfraredRadiationTypeConversion();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "ultrasonic_range_pdu_converter_test passed" << std::endl;
    return EXIT_SUCCESS;
}
