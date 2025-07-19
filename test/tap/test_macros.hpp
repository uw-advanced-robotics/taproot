#if __has_include("tap/errors/error_controller.hpp")
#define EXPECT_ERROR()\
    EXPECT_CALL(drivers.errorController, addToErrorList)
#define EXPECT_ERROR_TIMES(times)\
    EXPECT_ERROR().Times(times)
#else
#define EXPECT_ERROR()
#define EXPECT_ERROR_TIMES(times)
#endif

