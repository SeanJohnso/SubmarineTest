#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ping360_sonar_msgs__msg__SonarEcho

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SonarEcho {
    /// header info
    pub header: std_msgs::msg::Header,

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
    pub intensities: Vec<u8>,

}



impl Default for SonarEcho {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SonarEcho::default())
  }
}

impl rosidl_runtime_rs::Message for SonarEcho {
  type RmwMsg = super::msg::rmw::SonarEcho;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        angle: msg.angle,
        gain: msg.gain,
        number_of_samples: msg.number_of_samples,
        transmit_frequency: msg.transmit_frequency,
        speed_of_sound: msg.speed_of_sound,
        range: msg.range,
        intensities: msg.intensities.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      angle: msg.angle,
      gain: msg.gain,
      number_of_samples: msg.number_of_samples,
      transmit_frequency: msg.transmit_frequency,
      speed_of_sound: msg.speed_of_sound,
      range: msg.range,
        intensities: msg.intensities.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      angle: msg.angle,
      gain: msg.gain,
      number_of_samples: msg.number_of_samples,
      transmit_frequency: msg.transmit_frequency,
      speed_of_sound: msg.speed_of_sound,
      range: msg.range,
      intensities: msg.intensities
          .into_iter()
          .collect(),
    }
  }
}


