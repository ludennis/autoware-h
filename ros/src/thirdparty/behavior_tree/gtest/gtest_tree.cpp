/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <gtest/gtest.h>
#include <action_test_node.h>
#include <condition_test_node.h>
#include <behavior_tree.h>



struct SimpleSequenceTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action;
    std::shared_ptr<BT::ConditionTestNode> condition;

    std::shared_ptr<BT::SequenceNode> root;

    SimpleSequenceTest()
        : root(std::make_shared<BT::SequenceNode>("seq1"))
        , action(std::make_shared<BT::ActionTestNode>("action"))
        , condition(std::make_shared<BT::ConditionTestNode>("condition"))
    {
        root->AddChild(condition);
        root->AddChild(action);
    }

    ~SimpleSequenceTest()
    {
        std::cout << "~SimpleSequenceTest()" << '\n';
    }
};


struct ComplexSequenceTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;
    std::shared_ptr<BT::SequenceNode> seq_conditions;

    std::shared_ptr<BT::SequenceNode> root;
    ComplexSequenceTest()
        : root(std::make_shared<BT::SequenceNode>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
        , seq_conditions(std::make_shared<BT::SequenceNode>("seq_conditions"))
    {
        seq_conditions->AddChild(condition_1);
        seq_conditions->AddChild(condition_2);

        root->AddChild(seq_conditions);
        root->AddChild(action_1);
    }
};


struct ComplexSequence2ActionsTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::ActionTestNode> action_2;
    std::shared_ptr<BT::SequenceNode> seq_1;
    std::shared_ptr<BT::SequenceNode> seq_2;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::SequenceNode> root;

    ComplexSequence2ActionsTest()
        : root(std::make_shared<BT::SequenceNode>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , action_2(std::make_shared<BT::ActionTestNode>("action_2"))
        , seq_1(std::make_shared<BT::SequenceNode>("seq_1"))
        , seq_2(std::make_shared<BT::SequenceNode>("seq_2"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {

        seq_1->AddChild(condition_1);
        seq_1->AddChild(action_1);

        seq_2->AddChild(condition_2);
        seq_2->AddChild(action_2);

        root->AddChild(seq_1);
        root->AddChild(seq_2);
    }
};


struct SimpleFallbackTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action;
    std::shared_ptr<BT::ConditionTestNode> condition;
    std::shared_ptr<BT::FallbackNode> root;

    SimpleFallbackTest()
        : root(std::make_shared<BT::FallbackNode>("root"))
        , action(std::make_shared<BT::ActionTestNode>("action"))
        , condition(std::make_shared<BT::ConditionTestNode>("condition"))
    {
        root->AddChild(condition);
        root->AddChild(action);
    }
};


struct ComplexFallbackTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::FallbackNode> sel_conditions;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::FallbackNode> root;

    ComplexFallbackTest()
        : root(std::make_shared<BT::FallbackNode>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , sel_conditions(std::make_shared<BT::FallbackNode>("sel_conditions"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {
        sel_conditions->AddChild(condition_1);
        sel_conditions->AddChild(condition_2);
        root->AddChild(sel_conditions);
        root->AddChild(action_1);
    }
};

struct BehaviorTreeTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::FallbackNode> sel_conditions;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::SequenceNode> root;

    BehaviorTreeTest()
        : root(std::make_shared<BT::SequenceNode>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , sel_conditions(std::make_shared<BT::FallbackNode>("sel_conditions"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {
        sel_conditions->AddChild(condition_1);
        sel_conditions->AddChild(condition_2);

        root->AddChild(sel_conditions);
        root->AddChild(action_1);
    }
};

struct SimpleSequenceWithMemoryTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action;
    std::shared_ptr<BT::ConditionTestNode> condition;

    std::shared_ptr<BT::SequenceNodeWithMemory> root;

    SimpleSequenceWithMemoryTest()
        : root(std::make_shared<BT::SequenceNodeWithMemory>("seq1"))
        , action(std::make_shared<BT::ActionTestNode>("action"))
        , condition(std::make_shared<BT::ConditionTestNode>("condition"))
    {
        root->AddChild(condition);
        root->AddChild(action);
    }
};

struct ComplexSequenceWithMemoryTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::ActionTestNode> action_2;
    std::shared_ptr<BT::SequenceNodeWithMemory> seq_conditions;
    std::shared_ptr<BT::SequenceNodeWithMemory> seq_actions;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::SequenceNodeWithMemory> root;

    ComplexSequenceWithMemoryTest()
        : root(std::make_shared<BT::SequenceNodeWithMemory>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , action_2(std::make_shared<BT::ActionTestNode>("action_2"))
        , seq_conditions(std::make_shared<BT::SequenceNodeWithMemory>("sequence_conditions"))
        , seq_actions(std::make_shared<BT::SequenceNodeWithMemory>("sequence_actions"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {
        seq_actions->AddChild(action_1);
        seq_actions->AddChild(action_2);

        seq_conditions->AddChild(condition_1);
        seq_conditions->AddChild(condition_2);

        root->AddChild(seq_conditions);
        root->AddChild(seq_actions);
    }
};

struct SimpleFallbackWithMemoryTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action;
    std::shared_ptr<BT::ConditionTestNode> condition;

    std::shared_ptr<BT::FallbackNodeWithMemory> root;
    SimpleFallbackWithMemoryTest()
        : root(std::make_shared<BT::FallbackNodeWithMemory>("seq1"))
        , action(std::make_shared<BT::ActionTestNode>("action"))
        , condition(std::make_shared<BT::ConditionTestNode>("condition"))
    {
        root->AddChild(condition);
        root->AddChild(action);
    }
};

struct ComplexFallbackWithMemoryTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::ActionTestNode> action_2;
    std::shared_ptr<BT::FallbackNodeWithMemory> fal_conditions;
    std::shared_ptr<BT::FallbackNodeWithMemory> fal_actions;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::FallbackNodeWithMemory> root;

    ComplexFallbackWithMemoryTest()
        : root(std::make_shared<BT::FallbackNodeWithMemory>("root"))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , action_2(std::make_shared<BT::ActionTestNode>("action_2"))
        , fal_conditions(std::make_shared<BT::FallbackNodeWithMemory>("fallback_conditions"))
        , fal_actions(std::make_shared<BT::FallbackNodeWithMemory>("fallback_actions"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {
        fal_actions->AddChild(action_1);
        fal_actions->AddChild(action_2);

        fal_conditions->AddChild(condition_1);
        fal_conditions->AddChild(condition_2);

        root->AddChild(fal_conditions);
        root->AddChild(fal_actions);
    }
};

struct SimpleParallelTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::ActionTestNode> action_2;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;

    std::shared_ptr<BT::ParallelNode> root;

    SimpleParallelTest()
        : root(std::make_shared<BT::ParallelNode>("par", 4))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , action_2(std::make_shared<BT::ActionTestNode>("action_2"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
    {
        root->AddChild(condition_1);
        root->AddChild(action_1);
        root->AddChild(condition_2);
        root->AddChild(action_2);
    }
};

struct ComplexParallelTest : testing::Test
{
    std::shared_ptr<BT::ActionTestNode> action_1;
    std::shared_ptr<BT::ActionTestNode> action_2;
    std::shared_ptr<BT::ActionTestNode> action_3;

    std::shared_ptr<BT::ConditionTestNode> condition_1;
    std::shared_ptr<BT::ConditionTestNode> condition_2;
    std::shared_ptr<BT::ConditionTestNode> condition_3;

    std::shared_ptr<BT::ParallelNode> root;
    std::shared_ptr<BT::ParallelNode> parallel_1;
    std::shared_ptr<BT::ParallelNode> parallel_2;

    ComplexParallelTest()
        : root(std::make_shared<BT::ParallelNode>("root", 2))
        , parallel_1(std::make_shared<BT::ParallelNode>("par1", 3))
        , parallel_2(std::make_shared<BT::ParallelNode>("par1", 1))
        , action_1(std::make_shared<BT::ActionTestNode>("action_1"))
        , action_2(std::make_shared<BT::ActionTestNode>("action_2"))
        , action_3(std::make_shared<BT::ActionTestNode>("action_3"))
        , condition_1(std::make_shared<BT::ConditionTestNode>("condition_1"))
        , condition_2(std::make_shared<BT::ConditionTestNode>("condition_2"))
        , condition_3(std::make_shared<BT::ConditionTestNode>("condition_3"))
    {
        parallel_1->AddChild(condition_1);
        parallel_1->AddChild(action_1);
        parallel_1->AddChild(condition_2);
        parallel_1->AddChild(action_2);

        parallel_2->AddChild(condition_3);
        parallel_2->AddChild(action_3);

        root->AddChild(parallel_1);
        root->AddChild(parallel_2);
    }
};



/****************TESTS START HERE***************************/



TEST_F(SimpleSequenceTest, ConditionTrue)
{
    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    BT::ReturnStatus state = root->Tick();
    // std::cout << "/* message */" << '\n';
    ASSERT_EQ(BT::RUNNING, action->get_status());
    std::cout << "Checking action status" << '\n';
    ASSERT_EQ(BT::RUNNING, state);
    std::cout << "Checking root status" << '\n';
    root->Halt();
}


TEST_F(SimpleSequenceTest, ConditionTurnToFalse)
{
    BT::ReturnStatus state = root->Tick();
    condition->set_boolean_value(false);

    state = root->Tick();
    ASSERT_EQ(BT::FAILURE, state);
    ASSERT_EQ(BT::HALTED, action->get_status());
    root->Halt();
}


TEST_F(ComplexSequenceTest, ComplexSequenceConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, state);
    root->Halt();
}


TEST_F(ComplexSequence2ActionsTest, ConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    state = root->Tick();

    std::this_thread::sleep_for(std::chrono::seconds(5));
    state = root->Tick();
    state = root->Tick();


    ASSERT_EQ(BT::RUNNING, state);
    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, seq_1->get_status());
    ASSERT_EQ(BT::HALTED, seq_2->get_status());
    ASSERT_EQ(BT::HALTED, action_2->get_status());

    root->Halt();
}

TEST_F(ComplexSequenceTest, ComplexSequenceConditions1ToFalse)
{
    BT::ReturnStatus state = root->Tick();

    condition_1->set_boolean_value(false);

    state = root->Tick();

    ASSERT_EQ(BT::FAILURE, state);
    ASSERT_EQ(BT::HALTED, action_1->get_status());
    root->Halt();
}

TEST_F(ComplexSequenceTest, ComplexSequenceConditions2ToFalse)
{
    BT::ReturnStatus state = root->Tick();

    condition_2->set_boolean_value(false);

    state = root->Tick();

    ASSERT_EQ(BT::FAILURE, state);
    ASSERT_EQ(BT::HALTED, action_1->get_status());
    root->Halt();
}

TEST_F(SimpleFallbackTest, ConditionTrue)
{
    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    condition->set_boolean_value(true);
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, action->get_status());
    ASSERT_EQ(BT::SUCCESS, state);
    root->Halt();
}


TEST_F(SimpleFallbackTest, ConditionToFalse)
{
    condition->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();
    condition->set_boolean_value(true);


    state = root->Tick();

    ASSERT_EQ(BT::SUCCESS, state);
    ASSERT_EQ(BT::HALTED, action->get_status());
    root->Halt();
}


TEST_F(ComplexFallbackTest, Condition1ToTrue)
{
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();

    condition_1->set_boolean_value(true);

    state = root->Tick();

    ASSERT_EQ(BT::SUCCESS, state);

    ASSERT_EQ(BT::HALTED, action_1->get_status());
    root->Halt();
}

TEST_F(ComplexFallbackTest, Condition2ToTrue)
{
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();

    condition_2->set_boolean_value(true);

    state = root->Tick();

    ASSERT_EQ(BT::SUCCESS, state);
    ASSERT_EQ(BT::HALTED, action_1->get_status());
    root->Halt();
}



TEST_F(BehaviorTreeTest, Condition1ToFalseCondition2True)
{
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(true);

    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::RUNNING, state);
    ASSERT_EQ(BT::RUNNING, action_1->get_status());

    root->Halt();
}

TEST_F(BehaviorTreeTest, Condition2ToFalseCondition1True)
{
    condition_2->set_boolean_value(false);
    condition_1->set_boolean_value(true);

    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::RUNNING, state);
    ASSERT_EQ(BT::RUNNING, action_1->get_status());

    root->Halt();
}


TEST_F(SimpleSequenceWithMemoryTest, ConditionTrue)
{
    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    BT::ReturnStatus state = root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ASSERT_EQ(BT::RUNNING, action->get_status());
    ASSERT_EQ(BT::RUNNING, state);
    root->Halt();
}


TEST_F(SimpleSequenceWithMemoryTest, ConditionTurnToFalse)
{
    BT::ReturnStatus state = root->Tick();

    condition->set_boolean_value(false);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, state);
    ASSERT_EQ(BT::RUNNING, action->get_status());

    root->Halt();
}


TEST_F(ComplexSequenceWithMemoryTest, ConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}


TEST_F(ComplexSequenceWithMemoryTest, Conditions1ToFalse)
{
    BT::ReturnStatus state = root->Tick();

    condition_1->set_boolean_value(false);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);
    root->Halt();
}

TEST_F(ComplexSequenceWithMemoryTest, Conditions2ToFalse)
{
    BT::ReturnStatus state = root->Tick();

    condition_2->set_boolean_value(false);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(ComplexSequenceWithMemoryTest, Action1Done)
{
    root->Tick();

    condition_2->set_boolean_value(false);

    root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(10));
    root->Tick();

    ASSERT_EQ(BT::RUNNING, action_2->get_status());

    root->Halt();
}

TEST_F(SimpleFallbackWithMemoryTest, ConditionFalse)
{
    std::cout << "Ticking the root node !" << std::endl << std::endl;
    // Ticking the root node
    condition->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ASSERT_EQ(BT::RUNNING, action->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(SimpleFallbackWithMemoryTest, ConditionTurnToTrue)
{
    condition->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();
    condition->set_boolean_value(true);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, state);
    ASSERT_EQ(BT::RUNNING, action->get_status());

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, ConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::SUCCESS, state);

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, Condition1False)
{
    condition_1->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::SUCCESS, state);

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, ConditionsFalse)
{
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, Conditions1ToTrue)
{
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();
    condition_1->set_boolean_value(true);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, Conditions2ToTrue)
{
    condition_1->set_boolean_value(false);

    condition_2->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();

    condition_2->set_boolean_value(true);

    state = root->Tick();

    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(ComplexFallbackWithMemoryTest, Action1Failed)
{
    action_1->set_boolean_value(false);
    condition_1->set_boolean_value(false);
    condition_2->set_boolean_value(false);

    BT::ReturnStatus state = root->Tick();

    state = root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    state = root->Tick();

    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}

TEST_F(SimpleParallelTest, ConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}


TEST_F(SimpleParallelTest, Threshold_3)
{
    root->set_threshold_M(3);
    action_2->set_time(200);
    root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::HALTED, action_2->get_status());
    ASSERT_EQ(BT::SUCCESS, state);

    root->Halt();
}


TEST_F(SimpleParallelTest, Threshold_1)
{
    root->set_threshold_M(1);
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::IDLE, action_2->get_status());
    ASSERT_EQ(BT::SUCCESS, state);

    root->Halt();
}

TEST_F(ComplexParallelTest, ConditionsTrue)
{
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::IDLE, condition_3->get_status());
    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, action_2->get_status());
    ASSERT_EQ(BT::IDLE, action_3->get_status());
    ASSERT_EQ(BT::RUNNING, parallel_1->get_status());
    ASSERT_EQ(BT::IDLE, parallel_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}



TEST_F(ComplexParallelTest, Condition3False)
{
    condition_3->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();

    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::IDLE, condition_3->get_status());
    ASSERT_EQ(BT::RUNNING, action_1->get_status());
    ASSERT_EQ(BT::RUNNING, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, action_3->get_status());
    ASSERT_EQ(BT::RUNNING, parallel_1->get_status());
    ASSERT_EQ(BT::RUNNING, parallel_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);

    root->Halt();
}


TEST_F(ComplexParallelTest, Condition3FalseAction1Done)
{
    action_2->set_time(10);
    action_3->set_time(10);

    condition_3->set_boolean_value(false);
    BT::ReturnStatus state = root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(5));


    ASSERT_EQ(BT::IDLE, condition_1->get_status());
    ASSERT_EQ(BT::IDLE, condition_2->get_status());
    ASSERT_EQ(BT::IDLE, condition_3->get_status());
    ASSERT_EQ(BT::SUCCESS, action_1->get_status());  // success not read yet by the node parallel_1
    ASSERT_EQ(BT::RUNNING, parallel_1->get_status());  // parallel_1 hasn't realize (yet) that action_1 has succeeded

    state = root->Tick();

    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::IDLE, parallel_1->get_status());
    ASSERT_EQ(BT::HALTED, action_2->get_status());
    ASSERT_EQ(BT::RUNNING, action_3->get_status());
    ASSERT_EQ(BT::RUNNING, parallel_2->get_status());
    ASSERT_EQ(BT::RUNNING, state);


    state = root->Tick();
    std::this_thread::sleep_for(std::chrono::seconds(15));
    state = root->Tick();

    ASSERT_EQ(BT::IDLE, parallel_2->get_status());
    ASSERT_EQ(BT::IDLE, action_1->get_status());
    ASSERT_EQ(BT::IDLE, parallel_1->get_status());
    ASSERT_EQ(BT::IDLE, action_3->get_status());
    ASSERT_EQ(BT::IDLE, parallel_2->get_status());
    ASSERT_EQ(BT::SUCCESS, state);

    root->Halt();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
