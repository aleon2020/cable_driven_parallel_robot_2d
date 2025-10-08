// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tfg_interfaces:action/CountUntil.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tfg_interfaces/action/count_until.hpp"


#ifndef TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_
#define TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tfg_interfaces/action/detail/count_until__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_Goal_period
{
public:
  explicit Init_CountUntil_Goal_period(::tfg_interfaces::action::CountUntil_Goal & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_Goal period(::tfg_interfaces::action::CountUntil_Goal::_period_type arg)
  {
    msg_.period = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_Goal msg_;
};

class Init_CountUntil_Goal_target_number
{
public:
  Init_CountUntil_Goal_target_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_Goal_period target_number(::tfg_interfaces::action::CountUntil_Goal::_target_number_type arg)
  {
    msg_.target_number = std::move(arg);
    return Init_CountUntil_Goal_period(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_Goal>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_Goal_target_number();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_Result_reached_number
{
public:
  Init_CountUntil_Result_reached_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tfg_interfaces::action::CountUntil_Result reached_number(::tfg_interfaces::action::CountUntil_Result::_reached_number_type arg)
  {
    msg_.reached_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_Result>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_Result_reached_number();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_Feedback_current_number
{
public:
  Init_CountUntil_Feedback_current_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tfg_interfaces::action::CountUntil_Feedback current_number(::tfg_interfaces::action::CountUntil_Feedback::_current_number_type arg)
  {
    msg_.current_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_Feedback>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_Feedback_current_number();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Request_goal
{
public:
  explicit Init_CountUntil_SendGoal_Request_goal(::tfg_interfaces::action::CountUntil_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_SendGoal_Request goal(::tfg_interfaces::action::CountUntil_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Request msg_;
};

class Init_CountUntil_SendGoal_Request_goal_id
{
public:
  Init_CountUntil_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Request_goal goal_id(::tfg_interfaces::action::CountUntil_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CountUntil_SendGoal_Request_goal(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_SendGoal_Request>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_SendGoal_Request_goal_id();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Response_stamp
{
public:
  explicit Init_CountUntil_SendGoal_Response_stamp(::tfg_interfaces::action::CountUntil_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_SendGoal_Response stamp(::tfg_interfaces::action::CountUntil_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Response msg_;
};

class Init_CountUntil_SendGoal_Response_accepted
{
public:
  Init_CountUntil_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Response_stamp accepted(::tfg_interfaces::action::CountUntil_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_CountUntil_SendGoal_Response_stamp(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_SendGoal_Response>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_SendGoal_Response_accepted();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Event_response
{
public:
  explicit Init_CountUntil_SendGoal_Event_response(::tfg_interfaces::action::CountUntil_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_SendGoal_Event response(::tfg_interfaces::action::CountUntil_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Event msg_;
};

class Init_CountUntil_SendGoal_Event_request
{
public:
  explicit Init_CountUntil_SendGoal_Event_request(::tfg_interfaces::action::CountUntil_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_CountUntil_SendGoal_Event_response request(::tfg_interfaces::action::CountUntil_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CountUntil_SendGoal_Event_response(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Event msg_;
};

class Init_CountUntil_SendGoal_Event_info
{
public:
  Init_CountUntil_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Event_request info(::tfg_interfaces::action::CountUntil_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CountUntil_SendGoal_Event_request(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_SendGoal_Event>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_SendGoal_Event_info();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_GetResult_Request_goal_id
{
public:
  Init_CountUntil_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tfg_interfaces::action::CountUntil_GetResult_Request goal_id(::tfg_interfaces::action::CountUntil_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_GetResult_Request>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_GetResult_Request_goal_id();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_GetResult_Response_result
{
public:
  explicit Init_CountUntil_GetResult_Response_result(::tfg_interfaces::action::CountUntil_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_GetResult_Response result(::tfg_interfaces::action::CountUntil_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Response msg_;
};

class Init_CountUntil_GetResult_Response_status
{
public:
  Init_CountUntil_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_GetResult_Response_result status(::tfg_interfaces::action::CountUntil_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CountUntil_GetResult_Response_result(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_GetResult_Response>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_GetResult_Response_status();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_GetResult_Event_response
{
public:
  explicit Init_CountUntil_GetResult_Event_response(::tfg_interfaces::action::CountUntil_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_GetResult_Event response(::tfg_interfaces::action::CountUntil_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Event msg_;
};

class Init_CountUntil_GetResult_Event_request
{
public:
  explicit Init_CountUntil_GetResult_Event_request(::tfg_interfaces::action::CountUntil_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_CountUntil_GetResult_Event_response request(::tfg_interfaces::action::CountUntil_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CountUntil_GetResult_Event_response(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Event msg_;
};

class Init_CountUntil_GetResult_Event_info
{
public:
  Init_CountUntil_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_GetResult_Event_request info(::tfg_interfaces::action::CountUntil_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CountUntil_GetResult_Event_request(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_GetResult_Event>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_GetResult_Event_info();
}

}  // namespace tfg_interfaces


namespace tfg_interfaces
{

namespace action
{

namespace builder
{

class Init_CountUntil_FeedbackMessage_feedback
{
public:
  explicit Init_CountUntil_FeedbackMessage_feedback(::tfg_interfaces::action::CountUntil_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::tfg_interfaces::action::CountUntil_FeedbackMessage feedback(::tfg_interfaces::action::CountUntil_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_FeedbackMessage msg_;
};

class Init_CountUntil_FeedbackMessage_goal_id
{
public:
  Init_CountUntil_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_FeedbackMessage_feedback goal_id(::tfg_interfaces::action::CountUntil_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CountUntil_FeedbackMessage_feedback(msg_);
  }

private:
  ::tfg_interfaces::action::CountUntil_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::tfg_interfaces::action::CountUntil_FeedbackMessage>()
{
  return tfg_interfaces::action::builder::Init_CountUntil_FeedbackMessage_goal_id();
}

}  // namespace tfg_interfaces

#endif  // TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_
