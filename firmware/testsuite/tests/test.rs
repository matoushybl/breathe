#![no_std]
#![no_main]

use breathe_rs as _; // memory layout + panic handler

// See https://crates.io/crates/defmt-test/0.1.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    #[test]
    fn assert_true() {
        assert!(true)
    }

    #[test]
    fn assert_false() {
        assert!(false, "TODO: write actual tests")
    }
}
