// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from wadar_interfaces:msg/TagRelativeLocation.idl
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
#include "wadar_interfaces/msg/detail/tag_relative_location__struct.h"
#include "wadar_interfaces/msg/detail/tag_relative_location__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool wadar_interfaces__msg__tag_relative_location__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("wadar_interfaces.msg._tag_relative_location.TagRelativeLocation", full_classname_dest, 63) == 0);
  }
  wadar_interfaces__msg__TagRelativeLocation * ros_message = _ros_message;
  {  // relative_heading
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative_heading");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->relative_heading = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // alignment
    PyObject * field = PyObject_GetAttrString(_pymsg, "alignment");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->alignment = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * wadar_interfaces__msg__tag_relative_location__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TagRelativeLocation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("wadar_interfaces.msg._tag_relative_location");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TagRelativeLocation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  wadar_interfaces__msg__TagRelativeLocation * ros_message = (wadar_interfaces__msg__TagRelativeLocation *)raw_ros_message;
  {  // relative_heading
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->relative_heading);
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative_heading", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alignment
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->alignment);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alignment", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
