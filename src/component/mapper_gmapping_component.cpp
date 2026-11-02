// mapper_gmapping のコンポーネント。M0 では版を返すプロセスだけを持つ (コンテナとプロセスは M2、issue #5)
#include <string>

#include "juiz/juiz.h"
#include "mapper_gmapping/version.h"

static auto mapper_gmapping_version_manif = ProcessManifest("mapper_gmapping_version");
std::optional<std::string> mapper_gmapping_version() {
    return mapper_gmapping::version();
}
COMPONENT_PROCESS_FACTORY(mapper_gmapping_version_manif, mapper_gmapping_version);

auto component_manifest() {
    return ComponentManifest("mapper_gmapping")
        .add_process(mapper_gmapping_version_manifest());
}
DEFINE_COMPONENT_MANIFEST_ENTRY_POINT(component_manifest)
