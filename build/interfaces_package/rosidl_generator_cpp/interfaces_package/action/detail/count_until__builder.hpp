// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces_package:action/CountUntil.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces_package/action/count_until.hpp"


#ifndef INTERFACES_PACKAGE__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_
#define INTERFACES_PACKAGE__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces_package/action/detail/count_until__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_Goal_period
{
public:
  explicit Init_CountUntil_Goal_period(::interfaces_package::action::CountUntil_Goal & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_Goal period(::interfaces_package::action::CountUntil_Goal::_period_type arg)
  {
    msg_.period = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_Goal msg_;
};

class Init_CountUntil_Goal_target_number
{
public:
  Init_CountUntil_Goal_target_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_Goal_period target_number(::interfaces_package::action::CountUntil_Goal::_target_number_type arg)
  {
    msg_.target_number = std::move(arg);
    return Init_CountUntil_Goal_period(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_Goal>()
{
  return interfaces_package::action::builder::Init_CountUntil_Goal_target_number();
}

}  // namespace interfaces_package


namespace interfaces_package
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
  ::interfaces_package::action::CountUntil_Result reached_number(::interfaces_package::action::CountUntil_Result::_reached_number_type arg)
  {
    msg_.reached_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_Result>()
{
  return interfaces_package::action::builder::Init_CountUntil_Result_reached_number();
}

}  // namespace interfaces_package


namespace interfaces_package
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
  ::interfaces_package::action::CountUntil_Feedback current_number(::interfaces_package::action::CountUntil_Feedback::_current_number_type arg)
  {
    msg_.current_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_Feedback>()
{
  return interfaces_package::action::builder::Init_CountUntil_Feedback_current_number();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Request_goal
{
public:
  explicit Init_CountUntil_SendGoal_Request_goal(::interfaces_package::action::CountUntil_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_SendGoal_Request goal(::interfaces_package::action::CountUntil_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Request msg_;
};

class Init_CountUntil_SendGoal_Request_goal_id
{
public:
  Init_CountUntil_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Request_goal goal_id(::interfaces_package::action::CountUntil_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CountUntil_SendGoal_Request_goal(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_SendGoal_Request>()
{
  return interfaces_package::action::builder::Init_CountUntil_SendGoal_Request_goal_id();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Response_stamp
{
public:
  explicit Init_CountUntil_SendGoal_Response_stamp(::interfaces_package::action::CountUntil_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_SendGoal_Response stamp(::interfaces_package::action::CountUntil_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Response msg_;
};

class Init_CountUntil_SendGoal_Response_accepted
{
public:
  Init_CountUntil_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Response_stamp accepted(::interfaces_package::action::CountUntil_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_CountUntil_SendGoal_Response_stamp(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_SendGoal_Response>()
{
  return interfaces_package::action::builder::Init_CountUntil_SendGoal_Response_accepted();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_SendGoal_Event_response
{
public:
  explicit Init_CountUntil_SendGoal_Event_response(::interfaces_package::action::CountUntil_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_SendGoal_Event response(::interfaces_package::action::CountUntil_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Event msg_;
};

class Init_CountUntil_SendGoal_Event_request
{
public:
  explicit Init_CountUntil_SendGoal_Event_request(::interfaces_package::action::CountUntil_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_CountUntil_SendGoal_Event_response request(::interfaces_package::action::CountUntil_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CountUntil_SendGoal_Event_response(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Event msg_;
};

class Init_CountUntil_SendGoal_Event_info
{
public:
  Init_CountUntil_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_SendGoal_Event_request info(::interfaces_package::action::CountUntil_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CountUntil_SendGoal_Event_request(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_SendGoal_Event>()
{
  return interfaces_package::action::builder::Init_CountUntil_SendGoal_Event_info();
}

}  // namespace interfaces_package


namespace interfaces_package
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
  ::interfaces_package::action::CountUntil_GetResult_Request goal_id(::interfaces_package::action::CountUntil_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_GetResult_Request>()
{
  return interfaces_package::action::builder::Init_CountUntil_GetResult_Request_goal_id();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_GetResult_Response_result
{
public:
  explicit Init_CountUntil_GetResult_Response_result(::interfaces_package::action::CountUntil_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_GetResult_Response result(::interfaces_package::action::CountUntil_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Response msg_;
};

class Init_CountUntil_GetResult_Response_status
{
public:
  Init_CountUntil_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_GetResult_Response_result status(::interfaces_package::action::CountUntil_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_CountUntil_GetResult_Response_result(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_GetResult_Response>()
{
  return interfaces_package::action::builder::Init_CountUntil_GetResult_Response_status();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_GetResult_Event_response
{
public:
  explicit Init_CountUntil_GetResult_Event_response(::interfaces_package::action::CountUntil_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_GetResult_Event response(::interfaces_package::action::CountUntil_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Event msg_;
};

class Init_CountUntil_GetResult_Event_request
{
public:
  explicit Init_CountUntil_GetResult_Event_request(::interfaces_package::action::CountUntil_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_CountUntil_GetResult_Event_response request(::interfaces_package::action::CountUntil_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CountUntil_GetResult_Event_response(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Event msg_;
};

class Init_CountUntil_GetResult_Event_info
{
public:
  Init_CountUntil_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_GetResult_Event_request info(::interfaces_package::action::CountUntil_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CountUntil_GetResult_Event_request(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_GetResult_Event>()
{
  return interfaces_package::action::builder::Init_CountUntil_GetResult_Event_info();
}

}  // namespace interfaces_package


namespace interfaces_package
{

namespace action
{

namespace builder
{

class Init_CountUntil_FeedbackMessage_feedback
{
public:
  explicit Init_CountUntil_FeedbackMessage_feedback(::interfaces_package::action::CountUntil_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::interfaces_package::action::CountUntil_FeedbackMessage feedback(::interfaces_package::action::CountUntil_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_FeedbackMessage msg_;
};

class Init_CountUntil_FeedbackMessage_goal_id
{
public:
  Init_CountUntil_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CountUntil_FeedbackMessage_feedback goal_id(::interfaces_package::action::CountUntil_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_CountUntil_FeedbackMessage_feedback(msg_);
  }

private:
  ::interfaces_package::action::CountUntil_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces_package::action::CountUntil_FeedbackMessage>()
{
  return interfaces_package::action::builder::Init_CountUntil_FeedbackMessage_goal_id();
}

}  // namespace interfaces_package

#endif  // INTERFACES_PACKAGE__ACTION__DETAIL__COUNT_UNTIL__BUILDER_HPP_
