#ifndef MOCK_MACROS_HPP_
#define MOCK_MACROS_HPP_

#ifdef ENV_SIMULATOR
///< Wrap class functions that are not already virtual in this function if you wish to mock them.
#define mockable virtual
#else
#define mockable
#endif

#endif  // MOCK_MACROS_HPP_
