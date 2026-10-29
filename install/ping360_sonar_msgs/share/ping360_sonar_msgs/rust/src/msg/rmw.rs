#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ping360_sonar_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ping360_sonar_msgs__msg__SonarEcho() -> *const std::ffi::c_void;
}

#[link(name = "ping360_sonar_msgs__rosidl_generator_c")]
extern "C" {
    fn ping360_sonar_msgs__msg__SonarEcho__init(msg: *mut SonarEcho) -> bool;
    fn ping360_sonar_msgs__msg__SonarEcho__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SonarEcho>, size: usize) -> bool;
    fn ping360_sonar_msgs__msg__SonarEcho__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SonarEcho>);
    fn ping360_sonar_msgs__msg__SonarEcho__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SonarEcho>, out_seq: *mut rosidl_runtime_rs::Sequence<SonarEcho>) -> bool;
}

// Corresponds to ping360_sonar_msgs__msg__SonarEcho
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SonarEcho {
    /// header info
    pub header: std_msgs::msg::rmw::Header,

    /// the measurement angle
    pub angle: f32,

    /// Sonar Gain
    pub gain: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub number_of_samples: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub transmit_frequency: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub speed_of_sound: u16,

    /// range value
    pub range: u8,

    /// intensity data.  This is the actual data received from the sonar
    pub intensities: rosidl_runtime_rs::Sequence<u8>,

}



impl Default for SonarEcho {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ping360_sonar_msgs__msg__SonarEcho__init(&mut msg as *mut _) {
        panic!("Call to ping360_sonar_msgs__msg__SonarEcho__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SonarEcho {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ping360_sonar_msgs__msg__SonarEcho__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ping360_sonar_msgs__msg__SonarEcho__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ping360_sonar_msgs__msg__SonarEcho__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SonarEcho {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SonarEcho where Self: Sized {
  const TYPE_NAME: &'static str = "ping360_sonar_msgs/msg/SonarEcho";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ping360_sonar_msgs__msg__SonarEcho() }
  }
}


