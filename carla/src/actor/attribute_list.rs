//! Actor attribute list wrapper.

use std::collections::HashMap;

/// A list of actor attributes.
///
/// This wraps the attribute data from C++ in a more Rust-idiomatic way.
#[derive(Debug, Clone)]
pub struct AttributeList {
    /// The attributes as a map of key-value pairs
    attributes: HashMap<String, String>,
}

impl AttributeList {
    /// Create a new AttributeList from raw attribute strings.
    ///
    /// The strings are expected to be in "key=value" format.
    pub(crate) fn from_strings(attribute_strings: Vec<String>) -> Self {
        let mut attributes = HashMap::new();

        for attr_str in attribute_strings {
            if let Some((key, value)) = attr_str.split_once('=') {
                attributes.insert(key.to_string(), value.to_string());
            }
        }

        Self { attributes }
    }

    /// Get an attribute value by key.
    pub fn get(&self, key: &str) -> Option<&str> {
        self.attributes.get(key).map(|s| s.as_str())
    }

    /// Check if an attribute exists.
    pub fn contains(&self, key: &str) -> bool {
        self.attributes.contains_key(key)
    }

    /// Get the number of attributes.
    pub fn len(&self) -> usize {
        self.attributes.len()
    }

    /// Check if the attribute list is empty.
    pub fn is_empty(&self) -> bool {
        self.attributes.is_empty()
    }

    /// Get an iterator over all attribute keys.
    pub fn keys(&self) -> impl Iterator<Item = &str> {
        self.attributes.keys().map(|s| s.as_str())
    }

    /// Get an iterator over all attribute values.
    pub fn values(&self) -> impl Iterator<Item = &str> {
        self.attributes.values().map(|s| s.as_str())
    }

    /// Get an iterator over all attribute key-value pairs.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &str)> {
        self.attributes
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
    }

    /// Convert to a HashMap.
    pub fn to_map(&self) -> HashMap<String, String> {
        self.attributes.clone()
    }

    /// Try to parse an attribute value as a specific type.
    pub fn parse<T: std::str::FromStr>(&self, key: &str) -> Option<T> {
        self.get(key)?.parse().ok()
    }
}

impl IntoIterator for AttributeList {
    type Item = (String, String);
    type IntoIter = std::collections::hash_map::IntoIter<String, String>;

    fn into_iter(self) -> Self::IntoIter {
        self.attributes.into_iter()
    }
}

impl<'a> IntoIterator for &'a AttributeList {
    type Item = (&'a String, &'a String);
    type IntoIter = std::collections::hash_map::Iter<'a, String, String>;

    fn into_iter(self) -> Self::IntoIter {
        self.attributes.iter()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attribute_list_from_strings() {
        let strings = vec![
            "role_name=hero".to_string(),
            "generation=1".to_string(),
            "color=red".to_string(),
        ];

        let attr_list = AttributeList::from_strings(strings);

        assert_eq!(attr_list.len(), 3);
        assert_eq!(attr_list.get("role_name"), Some("hero"));
        assert_eq!(attr_list.get("generation"), Some("1"));
        assert_eq!(attr_list.get("color"), Some("red"));
        assert!(attr_list.contains("role_name"));
        assert!(!attr_list.contains("unknown"));
    }

    #[test]
    fn test_attribute_list_parse() {
        let strings = vec![
            "number=42".to_string(),
            "float=2.5".to_string(),
            "bool=true".to_string(),
        ];

        let attr_list = AttributeList::from_strings(strings);

        assert_eq!(attr_list.parse::<i32>("number"), Some(42));
        assert_eq!(attr_list.parse::<f64>("float"), Some(2.5));
        assert_eq!(attr_list.parse::<bool>("bool"), Some(true));
        assert_eq!(attr_list.parse::<i32>("unknown"), None);
    }

    #[test]
    fn test_attribute_list_iteration() {
        let strings = vec!["a=1".to_string(), "b=2".to_string()];

        let attr_list = AttributeList::from_strings(strings);

        let keys: Vec<_> = attr_list.keys().collect();
        assert!(keys.contains(&"a"));
        assert!(keys.contains(&"b"));

        let values: Vec<_> = attr_list.values().collect();
        assert!(values.contains(&"1"));
        assert!(values.contains(&"2"));

        let pairs: HashMap<_, _> = attr_list.iter().collect();
        assert_eq!(pairs.get("a"), Some(&"1"));
        assert_eq!(pairs.get("b"), Some(&"2"));
    }
}
