use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// Attribute macro that automatically manages CARLA server lifecycle for integration tests.
///
/// This macro transforms a test function to use automatic server management, providing
/// a connected CARLA client either as a parameter or in scope.
///
/// # Usage
///
/// ## Style 1: Explicit client parameter
/// ```rust
/// use carla_test_server::with_carla_server;
///
/// #[with_carla_server]
/// fn test_with_param(client: &carla::client::Client) {
///     let version = client.server_version().unwrap();
///     assert!(!version.is_empty());
/// }
/// ```
///
/// ## Style 2: Implicit client in scope
/// ```rust
/// use carla_test_server::with_carla_server;
///
/// #[with_carla_server]
/// fn test_with_scope() {
///     // client is automatically available
///     let version = client.server_version().unwrap();
///     assert!(!version.is_empty());
/// }
/// ```
#[proc_macro_attribute]
pub fn with_carla_server(_args: TokenStream, input: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(input as ItemFn);

    // Validate the function signature
    if !input_fn.sig.generics.params.is_empty() {
        return syn::Error::new_spanned(
            &input_fn.sig.generics,
            "Generic test functions are not supported by #[with_carla_server]",
        )
        .to_compile_error()
        .into();
    }

    if input_fn.sig.asyncness.is_some() {
        return syn::Error::new_spanned(
            &input_fn.sig,
            "Async test functions are not supported by #[with_carla_server]",
        )
        .to_compile_error()
        .into();
    }

    let fn_name = &input_fn.sig.ident;
    let fn_vis = &input_fn.vis;
    let fn_attrs = &input_fn.attrs;
    let fn_block = &input_fn.block;

    // Check if function has explicit client parameter
    let has_client_param = input_fn.sig.inputs.iter().any(|arg| {
        match arg {
            syn::FnArg::Typed(pat_type) => {
                // Check if the type is a reference
                if let syn::Type::Reference(type_ref) = &*pat_type.ty {
                    // Check if it's a path type
                    if let syn::Type::Path(type_path) = &*type_ref.elem {
                        // Check if it ends with "Client" or is fully qualified "carla::client::Client"
                        let path_str = type_path
                            .path
                            .segments
                            .iter()
                            .map(|seg| seg.ident.to_string())
                            .collect::<Vec<_>>()
                            .join("::");

                        return path_str == "Client"
                            || path_str == "carla::client::Client"
                            || path_str.ends_with("::Client");
                    }
                }
                false
            }
            _ => false,
        }
    });

    let expanded = if has_client_param {
        // Function expects client parameter
        quote! {
            #(#fn_attrs)*
            #[test]
            #fn_vis fn #fn_name() {
                carla_test_server::run_test_with_server(|client| {
                    let test_fn = |client: &carla::client::Client| #fn_block;
                    test_fn(client);
                }).expect("CARLA test failed");
            }
        }
    } else {
        // Function doesn't expect client parameter - inject into scope
        quote! {
            #(#fn_attrs)*
            #[test]
            #fn_vis fn #fn_name() {
                carla_test_server::run_test_with_server(|client| {
                    #[allow(unused_variables)]
                    let client = client; // Make available in scope
                    #fn_block
                }).expect("CARLA test failed");
            }
        }
    };

    TokenStream::from(expanded)
}
