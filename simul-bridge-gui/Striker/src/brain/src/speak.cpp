#include "brain.h"
#include "speak.h"
#include "brain_tree.h"

#include <cstdlib>
#include <ctime>

#define REGISTER_SPEAK_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [brain](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });


void RegisterSpeakNodes(BT::BehaviorTreeFactory &factory, Brain* brain){
    REGISTER_SPEAK_BUILDER(Speak)
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}
