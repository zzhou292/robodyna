#include "FamilySupport.h"

namespace robodyna::body_family_test { std::string box_file; }

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc != 2) return 2;
    robodyna::body_family_test::box_file=argv[1];
    return RUN_ALL_TESTS();
}
