// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ur_msgs:srv/SetForceMode.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ur_msgs/srv/detail/set_force_mode__struct.h"
#include "ur_msgs/srv/detail/set_force_mode__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__wrench__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__wrench__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__twist__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__twist__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ur_msgs__srv__set_force_mode__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ur_msgs.srv._set_force_mode.SetForceMode_Request", full_classname_dest, 48) == 0);
  }
  ur_msgs__srv__SetForceMode_Request * ros_message = _ros_message;
  {  // task_frame
    PyObject * field = PyObject_GetAttrString(_pymsg, "task_frame");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->task_frame)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // selection_vector_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_x");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_x = (Py_True == field);
    Py_DECREF(field);
  }
  {  // selection_vector_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_y");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_y = (Py_True == field);
    Py_DECREF(field);
  }
  {  // selection_vector_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_z");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_z = (Py_True == field);
    Py_DECREF(field);
  }
  {  // selection_vector_rx
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_rx");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_rx = (Py_True == field);
    Py_DECREF(field);
  }
  {  // selection_vector_ry
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_ry");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_ry = (Py_True == field);
    Py_DECREF(field);
  }
  {  // selection_vector_rz
    PyObject * field = PyObject_GetAttrString(_pymsg, "selection_vector_rz");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->selection_vector_rz = (Py_True == field);
    Py_DECREF(field);
  }
  {  // wrench
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrench");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__wrench__convert_from_py(field, &ros_message->wrench)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // type
    PyObject * field = PyObject_GetAttrString(_pymsg, "type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // speed_limits
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed_limits");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__twist__convert_from_py(field, &ros_message->speed_limits)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // deviation_limits
    PyObject * field = PyObject_GetAttrString(_pymsg, "deviation_limits");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
      Py_ssize_t size = 6;
      float * dest = ros_message->deviation_limits;
      for (Py_ssize_t i = 0; i < size; ++i) {
        float tmp = *(npy_float32 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(float));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // damping_factor
    PyObject * field = PyObject_GetAttrString(_pymsg, "damping_factor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->damping_factor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // gain_scaling
    PyObject * field = PyObject_GetAttrString(_pymsg, "gain_scaling");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->gain_scaling = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ur_msgs__srv__set_force_mode__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetForceMode_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ur_msgs.srv._set_force_mode");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetForceMode_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ur_msgs__srv__SetForceMode_Request * ros_message = (ur_msgs__srv__SetForceMode_Request *)raw_ros_message;
  {  // task_frame
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->task_frame);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "task_frame", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_x
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_x ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_y
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_y ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_z
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_z ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_rx
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_rx ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_rx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_ry
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_ry ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_ry", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // selection_vector_rz
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->selection_vector_rz ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "selection_vector_rz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wrench
    PyObject * field = NULL;
    field = geometry_msgs__msg__wrench__convert_to_py(&ros_message->wrench);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrench", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed_limits
    PyObject * field = NULL;
    field = geometry_msgs__msg__twist__convert_to_py(&ros_message->speed_limits);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed_limits", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // deviation_limits
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "deviation_limits");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT32);
    assert(sizeof(npy_float32) == sizeof(float));
    npy_float32 * dst = (npy_float32 *)PyArray_GETPTR1(seq_field, 0);
    float * src = &(ros_message->deviation_limits[0]);
    memcpy(dst, src, 6 * sizeof(float));
    Py_DECREF(field);
  }
  {  // damping_factor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->damping_factor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "damping_factor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gain_scaling
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->gain_scaling);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gain_scaling", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__struct.h"
// already included above
// #include "ur_msgs/srv/detail/set_force_mode__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ur_msgs__srv__set_force_mode__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ur_msgs.srv._set_force_mode.SetForceMode_Response", full_classname_dest, 49) == 0);
  }
  ur_msgs__srv__SetForceMode_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ur_msgs__srv__set_force_mode__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetForceMode_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ur_msgs.srv._set_force_mode");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetForceMode_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ur_msgs__srv__SetForceMode_Response * ros_message = (ur_msgs__srv__SetForceMode_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
