#include "src/MinimumSnapOsqp/minimumSnap.hpp"
#include <iostream>

// 未指定的途经点/端点速度会产生无界约束，必须仍然能够通过OSQP收敛。
int main(){
    const std::vector<Eigen::Vector2f> path{{1.f, 1.f}, {3.f, 2.f}, {5.f, 1.f}};
    MinimumSnap::Map map(200, 200);
    map.setConstant(255);
    bool passed = true;
    for(bool normalized : {false, true}){
        for(int scenario = 0; scenario < 4; scenario++){
            MinimumSnap solver;
            solver.setMap(map, 0.05f, 0.f, 0.f);
            solver.setOrder(6);
            solver.setMaxDx(3);
            MinimumSnap::SolveInput input;
            input.setNormTime(normalized);
            input.setCollisionCheckIter(2);
            input.setMaxCorridorRange(0.5f);
            input.setBackend(scenario == 2 ? MinimumSnap::Backend::OSQPCorridor :
                MinimumSnap::Backend::OSQPPath);
            if(scenario == 1 || scenario == 3){
                std::vector<MinimumSnap::SolveInput::Item> items(path.size());
                for(size_t i = 0; i < path.size(); i++){
                    items[i].xy = path[i];
                    // scenario=1: 所有速度自由；scenario=3: 所有速度固定。
                    items[i].useVxvy = scenario == 3;
                }
                input.setItems(items);
            }else{
                input.setPath(path);
                input.setInitVel({0.f, 0.f});
            }

            const auto result = solver.solve(input);
            bool ok = result.success && !result.path.empty();
            if(ok){
                ok = (result.path.front() - path.front()).norm() < 1e-4f &&
                    (result.path.back() - path.back()).norm() < 1e-4f;
                for(const auto& point : result.path){
                    ok = ok && point.allFinite();
                }
            }
            if(scenario != 2){
                input.setBackend(MinimumSnap::Backend::Close);
                const auto reference = solver.solve(input);
                ok = ok && reference.success && reference.path.size() == result.path.size();
                if(ok){
                    for(size_t i = 0; i < result.path.size(); i++){
                        ok = ok && (result.path[i] - reference.path[i]).norm() < 1e-3f;
                    }
                }
            }
            std::cout << "normalized=" << normalized << " scenario=" << scenario
                << " success=" << result.success << " points=" << result.path.size()
                << " check=" << (ok ? "PASS" : "FAIL") << std::endl;
            passed = passed && ok;
        }
    }
    return passed ? 0 : 1;
}
