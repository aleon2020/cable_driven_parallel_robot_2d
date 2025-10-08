// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tfg_interfaces:action/CountUntil.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tfg_interfaces/action/count_until.h"


#ifndef TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__FUNCTIONS_H_
#define TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "tfg_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "tfg_interfaces/action/detail/count_until__struct.h"

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil__get_type_hash(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil__get_type_description(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil__get_individual_type_description_source(
  const rosidl_action_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil__get_type_description_sources(
  const rosidl_action_type_support_t * type_support);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_Goal
 * )) before or use
 * tfg_interfaces__action__CountUntil_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__init(tfg_interfaces__action__CountUntil_Goal * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Goal__fini(tfg_interfaces__action__CountUntil_Goal * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Goal *
tfg_interfaces__action__CountUntil_Goal__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Goal__destroy(tfg_interfaces__action__CountUntil_Goal * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__are_equal(const tfg_interfaces__action__CountUntil_Goal * lhs, const tfg_interfaces__action__CountUntil_Goal * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__copy(
  const tfg_interfaces__action__CountUntil_Goal * input,
  tfg_interfaces__action__CountUntil_Goal * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_Goal__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_Goal__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_Goal__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_Goal__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__Sequence__init(tfg_interfaces__action__CountUntil_Goal__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Goal__Sequence__fini(tfg_interfaces__action__CountUntil_Goal__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Goal__Sequence *
tfg_interfaces__action__CountUntil_Goal__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Goal__Sequence__destroy(tfg_interfaces__action__CountUntil_Goal__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__Sequence__are_equal(const tfg_interfaces__action__CountUntil_Goal__Sequence * lhs, const tfg_interfaces__action__CountUntil_Goal__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Goal__Sequence__copy(
  const tfg_interfaces__action__CountUntil_Goal__Sequence * input,
  tfg_interfaces__action__CountUntil_Goal__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_Result
 * )) before or use
 * tfg_interfaces__action__CountUntil_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__init(tfg_interfaces__action__CountUntil_Result * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Result__fini(tfg_interfaces__action__CountUntil_Result * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Result *
tfg_interfaces__action__CountUntil_Result__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Result__destroy(tfg_interfaces__action__CountUntil_Result * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__are_equal(const tfg_interfaces__action__CountUntil_Result * lhs, const tfg_interfaces__action__CountUntil_Result * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__copy(
  const tfg_interfaces__action__CountUntil_Result * input,
  tfg_interfaces__action__CountUntil_Result * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_Result__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_Result__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_Result__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_Result__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__Sequence__init(tfg_interfaces__action__CountUntil_Result__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Result__Sequence__fini(tfg_interfaces__action__CountUntil_Result__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Result__Sequence *
tfg_interfaces__action__CountUntil_Result__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Result__Sequence__destroy(tfg_interfaces__action__CountUntil_Result__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__Sequence__are_equal(const tfg_interfaces__action__CountUntil_Result__Sequence * lhs, const tfg_interfaces__action__CountUntil_Result__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Result__Sequence__copy(
  const tfg_interfaces__action__CountUntil_Result__Sequence * input,
  tfg_interfaces__action__CountUntil_Result__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_Feedback
 * )) before or use
 * tfg_interfaces__action__CountUntil_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__init(tfg_interfaces__action__CountUntil_Feedback * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Feedback__fini(tfg_interfaces__action__CountUntil_Feedback * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Feedback *
tfg_interfaces__action__CountUntil_Feedback__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Feedback__destroy(tfg_interfaces__action__CountUntil_Feedback * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__are_equal(const tfg_interfaces__action__CountUntil_Feedback * lhs, const tfg_interfaces__action__CountUntil_Feedback * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__copy(
  const tfg_interfaces__action__CountUntil_Feedback * input,
  tfg_interfaces__action__CountUntil_Feedback * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_Feedback__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_Feedback__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_Feedback__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_Feedback__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__Sequence__init(tfg_interfaces__action__CountUntil_Feedback__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Feedback__Sequence__fini(tfg_interfaces__action__CountUntil_Feedback__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_Feedback__Sequence *
tfg_interfaces__action__CountUntil_Feedback__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_Feedback__Sequence__destroy(tfg_interfaces__action__CountUntil_Feedback__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__Sequence__are_equal(const tfg_interfaces__action__CountUntil_Feedback__Sequence * lhs, const tfg_interfaces__action__CountUntil_Feedback__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_Feedback__Sequence__copy(
  const tfg_interfaces__action__CountUntil_Feedback__Sequence * input,
  tfg_interfaces__action__CountUntil_Feedback__Sequence * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_SendGoal__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_SendGoal__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_SendGoal__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_SendGoal__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_SendGoal_Request
 * )) before or use
 * tfg_interfaces__action__CountUntil_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__init(tfg_interfaces__action__CountUntil_SendGoal_Request * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Request__fini(tfg_interfaces__action__CountUntil_SendGoal_Request * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Request *
tfg_interfaces__action__CountUntil_SendGoal_Request__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Request__destroy(tfg_interfaces__action__CountUntil_SendGoal_Request * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Request * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Request * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Request * input,
  tfg_interfaces__action__CountUntil_SendGoal_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_SendGoal_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_SendGoal_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_SendGoal_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__init(tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__fini(tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__destroy(tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * input,
  tfg_interfaces__action__CountUntil_SendGoal_Request__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_SendGoal_Response
 * )) before or use
 * tfg_interfaces__action__CountUntil_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__init(tfg_interfaces__action__CountUntil_SendGoal_Response * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Response__fini(tfg_interfaces__action__CountUntil_SendGoal_Response * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Response *
tfg_interfaces__action__CountUntil_SendGoal_Response__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Response__destroy(tfg_interfaces__action__CountUntil_SendGoal_Response * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Response * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Response * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Response * input,
  tfg_interfaces__action__CountUntil_SendGoal_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_SendGoal_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_SendGoal_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_SendGoal_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__init(tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__fini(tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__destroy(tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * input,
  tfg_interfaces__action__CountUntil_SendGoal_Response__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_SendGoal_Event
 * )) before or use
 * tfg_interfaces__action__CountUntil_SendGoal_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__init(tfg_interfaces__action__CountUntil_SendGoal_Event * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Event__fini(tfg_interfaces__action__CountUntil_SendGoal_Event * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Event *
tfg_interfaces__action__CountUntil_SendGoal_Event__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Event__destroy(tfg_interfaces__action__CountUntil_SendGoal_Event * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Event * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Event * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Event * input,
  tfg_interfaces__action__CountUntil_SendGoal_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_SendGoal_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_SendGoal_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_SendGoal_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__init(tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__fini(tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence *
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__destroy(tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__are_equal(const tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * lhs, const tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence__copy(
  const tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * input,
  tfg_interfaces__action__CountUntil_SendGoal_Event__Sequence * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_GetResult__get_type_hash(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_GetResult__get_type_description(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_GetResult__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_GetResult__get_type_description_sources(
  const rosidl_service_type_support_t * type_support);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_GetResult_Request
 * )) before or use
 * tfg_interfaces__action__CountUntil_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__init(tfg_interfaces__action__CountUntil_GetResult_Request * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Request__fini(tfg_interfaces__action__CountUntil_GetResult_Request * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Request *
tfg_interfaces__action__CountUntil_GetResult_Request__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Request__destroy(tfg_interfaces__action__CountUntil_GetResult_Request * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Request * lhs, const tfg_interfaces__action__CountUntil_GetResult_Request * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Request * input,
  tfg_interfaces__action__CountUntil_GetResult_Request * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_GetResult_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_GetResult_Request__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_GetResult_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__init(tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__fini(tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__destroy(tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * lhs, const tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Request__Sequence__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * input,
  tfg_interfaces__action__CountUntil_GetResult_Request__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_GetResult_Response
 * )) before or use
 * tfg_interfaces__action__CountUntil_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__init(tfg_interfaces__action__CountUntil_GetResult_Response * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Response__fini(tfg_interfaces__action__CountUntil_GetResult_Response * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Response *
tfg_interfaces__action__CountUntil_GetResult_Response__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Response__destroy(tfg_interfaces__action__CountUntil_GetResult_Response * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Response * lhs, const tfg_interfaces__action__CountUntil_GetResult_Response * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Response * input,
  tfg_interfaces__action__CountUntil_GetResult_Response * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_GetResult_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_GetResult_Response__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_GetResult_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__init(tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__fini(tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__destroy(tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * lhs, const tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Response__Sequence__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * input,
  tfg_interfaces__action__CountUntil_GetResult_Response__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_GetResult_Event
 * )) before or use
 * tfg_interfaces__action__CountUntil_GetResult_Event__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__init(tfg_interfaces__action__CountUntil_GetResult_Event * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Event__fini(tfg_interfaces__action__CountUntil_GetResult_Event * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Event *
tfg_interfaces__action__CountUntil_GetResult_Event__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Event__destroy(tfg_interfaces__action__CountUntil_GetResult_Event * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Event * lhs, const tfg_interfaces__action__CountUntil_GetResult_Event * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Event * input,
  tfg_interfaces__action__CountUntil_GetResult_Event * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_GetResult_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_GetResult_Event__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_GetResult_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__init(tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__fini(tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence *
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__destroy(tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__are_equal(const tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * lhs, const tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_GetResult_Event__Sequence__copy(
  const tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * input,
  tfg_interfaces__action__CountUntil_GetResult_Event__Sequence * output);

/// Initialize action/CountUntil message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tfg_interfaces__action__CountUntil_FeedbackMessage
 * )) before or use
 * tfg_interfaces__action__CountUntil_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__init(tfg_interfaces__action__CountUntil_FeedbackMessage * msg);

/// Finalize action/CountUntil message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_FeedbackMessage__fini(tfg_interfaces__action__CountUntil_FeedbackMessage * msg);

/// Create action/CountUntil message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_FeedbackMessage *
tfg_interfaces__action__CountUntil_FeedbackMessage__create(void);

/// Destroy action/CountUntil message.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_FeedbackMessage__destroy(tfg_interfaces__action__CountUntil_FeedbackMessage * msg);

/// Check for action/CountUntil message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__are_equal(const tfg_interfaces__action__CountUntil_FeedbackMessage * lhs, const tfg_interfaces__action__CountUntil_FeedbackMessage * rhs);

/// Copy a action/CountUntil message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__copy(
  const tfg_interfaces__action__CountUntil_FeedbackMessage * input,
  tfg_interfaces__action__CountUntil_FeedbackMessage * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_type_hash_t *
tfg_interfaces__action__CountUntil_FeedbackMessage__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
tfg_interfaces__action__CountUntil_FeedbackMessage__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource *
tfg_interfaces__action__CountUntil_FeedbackMessage__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
tfg_interfaces__action__CountUntil_FeedbackMessage__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of action/CountUntil messages.
/**
 * It allocates the memory for the number of elements and calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__init(tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__fini(tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * array);

/// Create array of action/CountUntil messages.
/**
 * It allocates the memory for the array and calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence *
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/CountUntil messages.
/**
 * It calls
 * tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
void
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__destroy(tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * array);

/// Check for action/CountUntil message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__are_equal(const tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * lhs, const tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/CountUntil messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_tfg_interfaces
bool
tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence__copy(
  const tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * input,
  tfg_interfaces__action__CountUntil_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TFG_INTERFACES__ACTION__DETAIL__COUNT_UNTIL__FUNCTIONS_H_
