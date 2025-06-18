#[cxx::bridge]
mod test {
    unsafe extern "C++" {
        type TestType;
        fn get_vector() -> Vec<*mut TestType>;
    }
}
EOF < /dev/null