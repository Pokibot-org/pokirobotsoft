#include <unity.h>
#include "speedgen.h"
#include "relative_obstacle_storing.h"

speedgen_obj_t obj;

void setUp(void)
{
    obj = speedgen_init_obj();
}

void tearDown(void)
{
    // clean stuff up here
}

void test_path_is_correctly_loaded(void)
{
    coordinates_t path[2] = {(coordinates_t){69, 420}, (coordinates_t){12, 13}};
    uint8_t err = speedgen_import_path(&obj, path, 1);
    TEST_ASSERT_EQUAL_MESSAGE(0 , err, "import path error");
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_path_is_correctly_loaded);
    return UNITY_END();
}