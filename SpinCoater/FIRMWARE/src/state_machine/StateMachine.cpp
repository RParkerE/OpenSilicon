#include "state_machine/StateMachine.h"

namespace StateMachine {

    StateMachine::StateMachine()
        : currentState_(Domain::MachineState::Idle)
    {}

    void StateMachine::postEvent(const Domain::Event& event) {
        // Will move to thread-safe mutex or queue later
        eventQueue_.push(event);
    }

    void StateMachine::update() {
        while (!eventQueue_.empty()) {
            Domain::Event ev = eventQueue_.front();
            eventQueue_.pop();

            Domain::MachineState next = determineNextState(currentState_, ev);
            if (next != currentState_) {
                applyTransition(next);
            }
        }
    }

    Domain::MachineState StateMachine::getCurrentState() const {
        return currentState_;
    }

    void StateMachine::setStateChangeCallback(StateChangeCallback cb) {
        stateChangeCallback_ = cb;
    }

    Domain::MachineState StateMachine::determineNextState(Domain::MachineState current, const Domain::Event& event) {
        switch (current) {
            case Domain::MachineState::Idle:
                if (event.type == Domain::EventType::StartRequested)
                    return Domain::MachineState::Running;
                break;

            case Domain::MachineState::Running:
                if (event.type == Domain::EventType::PauseRequested)
                    return Domain::MachineState::Paused;
                if (event.type == Domain::EventType::StopRequested)
                    return Domain::MachineState::Idle;
                if (event.type == Domain::EventType::FaultDetected)
                    return Domain::MachineState::Fault;
                if (event.type == Domain::EventType::ProcessCompleted)
                    return Domain::MachineState::Idle;
                break;

            case Domain::MachineState::Paused:
                if (event.type == Domain::EventType::ResumeRequested)
                    return Domain::MachineState::Running;
                if (event.type == Domain::EventType::StopRequested)
                    return Domain::MachineState::Idle;
                if (event.type == Domain::EventType::FaultDetected)
                    return Domain::MachineState::Fault;
                break;

            case Domain::MachineState::Fault:
                if (event.type == Domain::EventType::ResumeRequested)
                    return Domain::MachineState::Idle;
                if (event.type == Domain::EventType::ResetRequested)
                    return Domain::MachineState::Idle;
                break;

            default:
                break;
        }
        return current;
    }

    void StateMachine::applyTransition(Domain::MachineState newState) {
        currentState_ = newState;
        if (stateChangeCallback_) {
            stateChangeCallback_(newState);
        }
    }
}
