#include <gtest/gtest.h>

#include <stdexcept>

#include "joystick_mapper/joystick_mapper.hpp"

TEST(ActiveModeTopic, SeparatesNamesAndKeepsLegacyDefault)
{
  EXPECT_EQ(joystick_mapper::activeModeTopic(""), "/joystick_mapper/active_mode");
  EXPECT_EQ(joystick_mapper::activeModeTopic("mouse"), "/joystick_mapper/mouse/active_mode");
  EXPECT_EQ(joystick_mapper::activeModeTopic("hand_2"), "/joystick_mapper/hand_2/active_mode");
}

TEST(ActiveModeTopic, RejectsInvalidSegments)
{
  for (const auto *name : {"3mouse", "mouse/hand", "mouse-hand", "hello world"})
    EXPECT_THROW(joystick_mapper::activeModeTopic(name), std::invalid_argument);
}
