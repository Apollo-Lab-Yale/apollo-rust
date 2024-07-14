use serde::de::DeserializeOwned;
use serde::Serialize;

pub trait ToRonString: Serialize {
    fn to_ron_string(&self) -> String {
        ron::to_string(self).expect("error")
    }
}
impl<T> ToRonString for T where T: Serialize { }
pub trait FromRonString: ToRonString + DeserializeOwned {
    fn from_ron_string(ron_str: &str) -> Self where Self: Sized {
        let load: Result<Self, _> = ron::from_str(ron_str);
        return if let Ok(load) = load { load } else {
            panic!("Could not load ron string {:?} into correct type.", ron_str)
        }
    }

    fn from_ron_string_option(ron_str: &str) -> Option<Self> where Self: Sized {
        let load: Result<Self, _> = ron::from_str(ron_str);
        return if let Ok(load) = load { Some(load) } else {
            None
        }
    }
}
impl<T> FromRonString for T where T: ToRonString + DeserializeOwned { }

pub trait ToJsonString: Serialize {
    fn to_json_string(&self) -> String {
        serde_json::to_string(self).expect("error")
    }
}
impl<T> ToJsonString for T where T: Serialize { }
pub trait FromJsonString: ToJsonString + DeserializeOwned {
    fn from_json_string(json_str: &str) -> Self where Self: Sized {
        let load: Result<Self, _> = serde_json::from_str(json_str);
        return if let Ok(load) = load { load } else {
            // Err(OptimaError::new_generic_error_str(&format!(), file!(), line!()))
            panic!("Could not load json string {:?} into correct type.", json_str);
        }
    }

    fn from_json_string_option(json_str: &str) -> Option<Self> where Self: Sized {
        let load: Result<Self, _> = serde_json::from_str(json_str);
        return if let Ok(load) = load { Some(load) } else {
            None
        }
    }
}
impl<T> FromJsonString for T where T: ToJsonString + DeserializeOwned { }

pub trait ToTomlString: Serialize {
    fn to_toml_string(&self) -> String {
        toml::to_string(self).expect("error")
    }
}
impl<T> ToTomlString for T where T: Serialize { }
pub trait FromTomlString: ToTomlString + DeserializeOwned {
    fn from_toml_string(toml_string: &str) -> Self where Self: Sized {
        let load: Result<Self, _> = toml::from_str(toml_string);
        return if let Ok(load) = load { load } else {
            // Err(OptimaError::new_generic_error_str(&format!("Could not load toml string {:?} into correct type.", toml_string), file!(), line!()))
            panic!("Could not load toml string {:?} into correct type.", toml_string);
        }
    }

    fn from_toml_string_option(toml_string: &str) -> Option<Self> where Self: Sized {
        let load: Result<Self, _> = toml::from_str(toml_string);
        return if let Ok(load) = load { Some(load) } else {
            None
        }
    }
}
impl<T> FromTomlString for T where T: ToTomlString + DeserializeOwned { }

pub trait ToYamlString: Serialize {
    fn to_yaml_string(&self) -> String { serde_yaml::to_string(self).expect("error") }
}
impl<T> ToYamlString for T where T: Serialize { }
pub trait FromYamlString: ToYamlString + DeserializeOwned {
    fn from_yaml_string(yaml_string: &str) -> Self where Self: Sized {
        let load: Result<Self, _> = serde_yaml::from_str(yaml_string);
        return if let Ok(load) = load { load } else {
            // Err(OptimaError::new_generic_error_str(&format!("Could not load toml string {:?} into correct type.", toml_string), file!(), line!()))
            panic!("Could not load toml string {:?} into correct type.", yaml_string);
        }
    }

    fn from_yaml_string_option(yaml_string: &str) -> Option<Self> where Self: Sized {
        let load: Result<Self, _> = serde_yaml::from_str(yaml_string);
        return if let Ok(load) = load { Some(load) } else {
            None
        }
    }
}
impl<T> FromYamlString for T where T: ToTomlString + DeserializeOwned { }