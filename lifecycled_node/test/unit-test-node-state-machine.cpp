#include <gtest/gtest.h>

#include "lifecycled_node/NodeStateMachine.h"

using lifecycled_node::NodeStateMachine;
using State = lifecycle_msgs::cpp::NodeStatus::State;

TEST (FunctionTest, DefaultState)
{
    NodeStateMachine machine;

    EXPECT_EQ(machine.currentState(), State::NONE);
}

TEST (FunctionTest, StateNone)
{
    NodeStateMachine machine{State::NONE};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::UNCONFIGURED));
    EXPECT_FALSE(machine.canChangeTo(State::INACTIVE));
    EXPECT_FALSE(machine.canChangeTo(State::ACTIVE));
    EXPECT_FALSE(machine.canChangeTo(State::FINALIZED));

    ASSERT_FALSE(machine.changeTo(State::UNCONFIGURED));
    ASSERT_FALSE(machine.changeTo(State::INACTIVE));
    ASSERT_FALSE(machine.changeTo(State::ACTIVE));
    ASSERT_FALSE(machine.changeTo(State::FINALIZED));

    // Change to state unconfigured.
    EXPECT_TRUE(machine.canChangeTo(State::CREATED));
    ASSERT_TRUE(machine.changeTo(State::CREATED));
}

TEST (FunctionTest, StateCreated)
{
    NodeStateMachine machine{State::CREATED};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::INACTIVE));
    EXPECT_FALSE(machine.canChangeTo(State::ACTIVE));

    ASSERT_FALSE(machine.changeTo(State::INACTIVE));
    ASSERT_FALSE(machine.changeTo(State::ACTIVE));

    // Change to state unconfigured.
    EXPECT_TRUE(machine.canChangeTo(State::UNCONFIGURED));
    ASSERT_TRUE(machine.changeTo(State::UNCONFIGURED));

    // Reset the state machine.
    machine = NodeStateMachine{State::CREATED};

    // Change to state finialized.
    EXPECT_TRUE(machine.canChangeTo(State::FINALIZED));
    ASSERT_TRUE(machine.changeTo(State::FINALIZED));
}

TEST (FunctionTest, StateUnconfigured)
{
    NodeStateMachine machine{State::UNCONFIGURED};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::CREATED));
    EXPECT_FALSE(machine.canChangeTo(State::ACTIVE));

    ASSERT_FALSE(machine.changeTo(State::CREATED));
    ASSERT_FALSE(machine.changeTo(State::ACTIVE));

    // Change to state inactive.
    EXPECT_TRUE(machine.canChangeTo(State::INACTIVE));
    ASSERT_TRUE(machine.changeTo(State::INACTIVE));

    // Reset the state machine.
    machine = NodeStateMachine{State::UNCONFIGURED};

    // Change to state finialized.
    EXPECT_TRUE(machine.canChangeTo(State::FINALIZED));
    ASSERT_TRUE(machine.changeTo(State::FINALIZED));
}

TEST (FunctionTest, StateInactive)
{
    NodeStateMachine machine{State::INACTIVE};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::CREATED));
    ASSERT_FALSE(machine.changeTo(State::CREATED));

    // Change to state active.
    EXPECT_TRUE(machine.canChangeTo(State::ACTIVE));
    ASSERT_TRUE(machine.changeTo(State::ACTIVE));

    // Reset the state machine.
    machine = NodeStateMachine{State::INACTIVE};

    // Change to state unconfigured.
    EXPECT_TRUE(machine.canChangeTo(State::UNCONFIGURED));
    ASSERT_TRUE(machine.changeTo(State::UNCONFIGURED));

    // Reset the state machine.
    machine = NodeStateMachine{State::INACTIVE};

    // Change to state finialized.
    EXPECT_TRUE(machine.canChangeTo(State::FINALIZED));
    ASSERT_TRUE(machine.changeTo(State::FINALIZED));
}

TEST (FunctionTest, StateActive)
{
    NodeStateMachine machine{State::ACTIVE};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::CREATED));
    EXPECT_FALSE(machine.canChangeTo(State::UNCONFIGURED));

    ASSERT_FALSE(machine.changeTo(State::CREATED));
    ASSERT_FALSE(machine.changeTo(State::UNCONFIGURED));

    // Change to state inactive.
    EXPECT_TRUE(machine.canChangeTo(State::INACTIVE));
    ASSERT_TRUE(machine.changeTo(State::INACTIVE));

    // Reset the state machine.
    machine = NodeStateMachine{State::INACTIVE};

    // Change to state finialized.
    EXPECT_TRUE(machine.canChangeTo(State::FINALIZED));
    ASSERT_TRUE(machine.changeTo(State::FINALIZED));

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::CREATED));
    EXPECT_FALSE(machine.canChangeTo(State::UNCONFIGURED));
}

TEST (FunctionTest, StateFinalized)
{
    NodeStateMachine machine{State::FINALIZED};

    // Test if all unvalid states will be rejected.
    EXPECT_FALSE(machine.canChangeTo(State::CREATED));
    EXPECT_FALSE(machine.canChangeTo(State::UNCONFIGURED));
    EXPECT_FALSE(machine.canChangeTo(State::INACTIVE));
    EXPECT_FALSE(machine.canChangeTo(State::ACTIVE));

    ASSERT_FALSE(machine.changeTo(State::CREATED));
    ASSERT_FALSE(machine.changeTo(State::UNCONFIGURED));
    ASSERT_FALSE(machine.changeTo(State::INACTIVE));
    ASSERT_FALSE(machine.changeTo(State::ACTIVE));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
