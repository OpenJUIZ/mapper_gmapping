// mapper_gmapping_core のテスト。M0 ではリンクできることだけを確かめる (本体のテストは M1、issue #4)
#include <cstdio>
#include <cstdlib>

#include <gmapping/gridfastslam/gridslamprocessor.h>

#include "mapper_gmapping/version.h"

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, \
                         __LINE__, #cond);                                \
            std::exit(1);                                                 \
        }                                                                 \
    } while (0)

int main() {
    CHECK(!mapper_gmapping::version().empty());

    // GMapping がリンクされ、プロセッサを作って捨てられる
    auto* processor = new GMapping::GridSlamProcessor();
    CHECK(processor->getParticles().empty());
    delete processor;

    std::puts("ok");
    return 0;
}
