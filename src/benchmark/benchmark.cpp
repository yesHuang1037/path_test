#include "benchmark.hpp"
#include "../YAstar/yastar.hpp"
#include "../YAstar/kinodynamicAstar.hpp"
#include "../MinimumSnapOsqp/minimumSnap.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

#ifndef BENCHMARK_SOURCE_DIR
#define BENCHMARK_SOURCE_DIR "."
#endif

namespace benchmark {
namespace {
using Clock = std::chrono::steady_clock;
using Point = Eigen::Vector2f;
using Item = MinimumSnap::SolveInput::Item;
const std::vector<std::string> stages{"initialize_ms", "cost_map_ms", "search_ms",
    "simplify_ms", "optimize_ms", "validate_ms"};
struct Options {
    std::string map = std::string(BENCHMARK_SOURCE_DIR) + "/images/rmuc2025.png";
    std::string mode = "demo", suite = "existing", output, inputs;
    double resolution = .05, timeLimit = 10., maxSpeed = 3., maxAcc = 1.;
    double corridorRange = 2.5, corridorShrink = 0., simplifyThreshold = .1;
    int count = 1000, maxIterations = 6, order = 6, maxDerivative = 3;
    uint32_t seed = static_cast<uint32_t>(std::time(nullptr));
    std::set<int> ids;
};
struct Case {
    int id = 0;
    std::string group, backend;
    Point start, end;
    std::vector<Item> items;
};
struct Record {
    Case input;
    bool passed = false, frontendSuccess = false, backendAttempted = false;
    std::string reason = "input_invalid";
    std::map<std::string, double> timings;
    double total = 0.;
    int collisions = 0;
    MinimumSnap::SolveOutput solve;
    std::vector<Point> front, path;
    std::vector<Eigen::Vector2d> velocity;
    std::vector<Eigen::Vector2d> acceleration;
    std::vector<double> trajectoryTime;
};
double milliseconds(Clock::time_point begin) {
    return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}
template<class F> void measure(Record& r, const std::string& name, F f) {
    auto begin = Clock::now();
    try { f(); } catch (...) { r.timings[name] = milliseconds(begin); throw; }
    r.timings[name] = milliseconds(begin);
}
void mkdirs(const std::string& path) {
    if(path.empty()) return;
    size_t pos = 0;
    do {
        pos = path.find('/', pos + 1);
        const auto part = path.substr(0, pos);
        if(!part.empty() && ::mkdir(part.c_str(), 0775) != 0 && errno != EEXIST)
            throw std::runtime_error("Cannot create directory: " + part);
        struct stat info;
        if(!part.empty() && (::stat(part.c_str(), &info) != 0 || !S_ISDIR(info.st_mode)))
            throw std::runtime_error("Not a directory: " + part);
    } while(pos != std::string::npos);
}
cv::FileStorage writer(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if(!fs.isOpened()) throw std::runtime_error("Cannot write: " + path);
    return fs;
}
void writeConfig(cv::FileStorage& fs, const Options& o) {
    fs << "schema_version" << 1 << "algorithm_revision" << 1 << "map" << o.map
       << "seed" << std::to_string(o.seed) << "count" << o.count << "suite" << o.suite
       << "resolution" << o.resolution << "time_limit" << o.timeLimit
       << "max_iterations" << o.maxIterations << "max_speed" << o.maxSpeed
       << "max_acc" << o.maxAcc << "order" << o.order << "max_derivative" << o.maxDerivative
       << "corridor_range" << o.corridorRange << "corridor_shrink" << o.corridorShrink
       << "simplify_threshold" << o.simplifyThreshold
       << "strict_collision" << 0 << "normalize_time" << 1
       << "cpp_compiler" << __VERSION__ << "opencv_version" << CV_VERSION
       << "osqp_max_iterations" << 20000 << "osqp_requested_time_limit" << .1
       << "osqp_absolute_tolerance" << 1e-8 << "osqp_relative_tolerance" << 1e-8
       << "speed_max_speed" << 8. << "speed_max_acc" << 2.
       << "kinodynamic" << "{" << "max_vel" << 4. << "max_acc" << 4.
       << "max_tau" << 2. << "vel_resolution" << .5 << "time_weight" << 1.
       << "heuristic_weight" << 8. << "sample_time" << .01 << "max_nodes" << 150000 << "}";
}
void readConfig(const cv::FileStorage& fs, Options& o) {
    if(static_cast<int>(fs["schema_version"]) != 1 || static_cast<int>(fs["algorithm_revision"]) != 1)
        throw std::runtime_error("Unsupported saved input version");
    fs["map"] >> o.map;
    std::string seed; fs["seed"] >> seed; o.seed = static_cast<uint32_t>(std::stoul(seed));
    fs["count"] >> o.count; fs["suite"] >> o.suite;
    fs["resolution"] >> o.resolution; fs["time_limit"] >> o.timeLimit;
    fs["max_iterations"] >> o.maxIterations; fs["max_speed"] >> o.maxSpeed;
    fs["max_acc"] >> o.maxAcc; fs["order"] >> o.order; fs["max_derivative"] >> o.maxDerivative;
    fs["corridor_range"] >> o.corridorRange; fs["corridor_shrink"] >> o.corridorShrink;
    fs["simplify_threshold"] >> o.simplifyThreshold;
}
void writeInputs(const std::string& path, const Options& o, const std::vector<Case>& cases,
                 const cv::Mat& map) {
    auto fs = writer(path);
    writeConfig(fs, o);
    fs << "width" << map.cols << "height" << map.rows << "coordinate_system" << "internal_x_right_y_down_meters";
    fs << "cases" << "[";
    for(const auto& c : cases) {
        fs << "{" << "id" << c.id << "group" << c.group << "backend" << c.backend
           << "start_x" << c.start.x() << "start_y" << c.start.y()
           << "end_x" << c.end.x() << "end_y" << c.end.y() << "items" << "[";
        for(const auto& item : c.items) {
            fs << "{" << "x" << item.xy.x() << "y" << item.xy.y()
               << "vx" << item.vxvy.x() << "vy" << item.vxvy.y()
               << "use_velocity" << static_cast<int>(item.useVxvy)
               << "auto_corridor" << static_cast<int>(item.autoCorridor) << "corridor" << "[";
            for(float value : item.corridor) fs << value;
            fs << "]" << "}";
        }
        fs << "]" << "}";
    }
    fs << "]";
}
std::vector<Case> readCases(const cv::FileStorage& fs) {
    std::vector<Case> cases;
    for(const auto& node : fs["cases"]) {
        Case c;
        node["id"] >> c.id; node["group"] >> c.group; node["backend"] >> c.backend;
        node["start_x"] >> c.start.x(); node["start_y"] >> c.start.y();
        node["end_x"] >> c.end.x(); node["end_y"] >> c.end.y();
        for(const auto& n : node["items"]) {
            Item item;
            n["x"] >> item.xy.x(); n["y"] >> item.xy.y();
            n["vx"] >> item.vxvy.x(); n["vy"] >> item.vxvy.y();
            item.useVxvy = static_cast<int>(n["use_velocity"]) != 0;
            item.autoCorridor = static_cast<int>(n["auto_corridor"]) != 0;
            int i = 0;
            for(const auto& v : n["corridor"]) {
                if(i >= 4) throw std::runtime_error("Invalid saved corridor");
                item.corridor[i++] = static_cast<float>(v);
            }
            if(i != 4) throw std::runtime_error("Invalid saved corridor");
            c.items.push_back(item);
        }
        if(c.group != "random" && c.group != "fixed" && c.group != "speed" && c.group != "kinodynamic")
            throw std::runtime_error("Unknown saved case group");
        if(c.backend != "OSQPCorridor" && c.backend != "Close" && c.backend != "KinodynamicAstar")
            throw std::runtime_error("Unknown saved backend");
        cases.push_back(c);
    }
    return cases;
}
std::vector<Case> generate(const Options& o, const cv::Mat& map) {
    std::vector<Case> cases;
    auto add = [&](const std::string& group, const std::string& backend, Point start, Point end) {
        Case c; c.id = static_cast<int>(cases.size()) + 1; c.group = group;
        c.backend = backend; c.start = start; c.end = end; cases.push_back(c);
    };
    auto legacy = [&](Point p) {
        p.y() = (map.rows - 1 - static_cast<int>(p.y() * (1.f/static_cast<float>(o.resolution)))) * o.resolution;
        return p;
    };
    if(o.suite == "existing") {
        Point start(25.2695f, 14.14615f), end(1.47266f, 9.1749f);
        start.y() = (map.rows - 1) * o.resolution - start.y();
        end.y() = (map.rows - 1) * o.resolution - end.y();
        add("kinodynamic", "KinodynamicAstar", start, end);
        const std::vector<Point> path{{25.25f,.85f},{24.85f,5.15f},{18.25f,4.25f},
            {16.3f,5.25f},{16.25f,5.25f},{15.95f,5.55f},{15.15f,9.8f},
            {11.9f,9.65f},{9.75f,10.75f},{4.5f,8.4f},{3.5f,6.1f},{1.45f,5.8f}};
        for(const auto& backend : {"Close", "OSQPCorridor"}) {
            add("speed", backend, path.front(), path.back());
            auto& c = cases.back();
            for(const auto& p : path) { Item item; item.xy = p; c.items.push_back(item); }
            for(int i : {1,6}) {
                auto& item = c.items[i]; item.useVxvy = true; item.autoCorridor = false;
                item.vxvy = i == 1 ? Point(-1.f,1.5f) : Point(.5f,2.f);
                item.corridor = {item.xy.x(),item.xy.y(),item.xy.x(),item.xy.y()};
            }
        }
        const std::vector<std::pair<Point,Point>> pairs{
            {{25.2695f,14.14615f},{1.47266f,9.1749f}},{{2.5323f,1.88688f},{25.5733f,13.5454f}},
            {{21.5323f,4.88688f},{4.1733f,14.1454f}},{{4.f,8.f},{7.f,.95f}},
            {{19.2695f,8.54615f},{1.47266f,9.1749f}},{{4.29f,13.05f},{23.7f,.95f}}};
        for(const auto& pair : pairs) add("fixed", "OSQPCorridor", legacy(pair.first), legacy(pair.second));
    }
    std::vector<cv::Point> free;
    for(int row=0; row<map.rows; ++row)
        for(int col=0; col<map.cols; ++col)
            if(map.at<unsigned char>(row,col) != 0) free.emplace_back(col,row);
    if(o.count && free.size() < 2) throw std::runtime_error("At least two free pixels are required");
    std::mt19937 gen(o.seed);
    if(o.count) {
        std::uniform_int_distribution<size_t> first(0, free.size()-1), second(0, free.size()-2);
        for(int i=0; i<o.count; ++i) {
            const size_t a = first(gen); size_t b = second(gen); if(b >= a) ++b;
            // Cell centers avoid floating-point truncation into a neighboring obstacle cell.
            auto point = [&](size_t n) { return Point((free[n].x+.5)*o.resolution, (free[n].y+.5)*o.resolution); };
            add("random", "OSQPCorridor", point(a), point(b));
        }
    }
    return cases;
}
bool inside(const Point& p, const Options& o, const cv::Mat& map) {
    return p.allFinite() && p.x() >= 0 && p.y() >= 0 &&
        p.x() < map.cols*o.resolution && p.y() < map.rows*o.resolution;
}
void configure(MinimumSnap& solver, MinimumSnap::SolveInput& input, const Options& o,
               const MinimumSnap::Map& map) {
    solver.setOrder(o.order); solver.setMaxDx(o.maxDerivative); solver.setDt(.1f);
    solver.setMap(map, o.resolution, 0, 0); solver.setTL(o.timeLimit);
    input.setCollisionCheckIter(o.maxIterations); input.setMaxSpeed(o.maxSpeed); input.setMaxAcc(o.maxAcc);
    input.setMaxCorridorRange(o.corridorRange); input.setCorridorShrink(o.corridorShrink); input.setNormTime(true);
}
Record execute(const Case& c, const Options& o, const cv::Mat& map) {
    Record r; r.input = c;
    auto begin = Clock::now();
    try {
        if(!inside(c.start,o,map) || !inside(c.end,o,map)) throw std::runtime_error("input_out_of_map");
        if(c.group == "kinodynamic") {
            std::unique_ptr<KinodynamicAstar> astar;
            measure(r,"initialize_ms",[&] {
                astar.reset(new KinodynamicAstar(map.cols,map.rows,o.resolution,0,0));
                astar->setMap(map.cols,map.rows,const_cast<unsigned char*>(map.ptr<unsigned char>()));
                astar->config.maxNodes = 150000; astar->config.sampleTime = .01;
            });
            KinodynamicAstar::Result result;
            measure(r,"search_ms",[&] { result = astar->search(c.start.cast<double>(),
                Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero(),c.end.cast<double>()); });
            r.frontendSuccess = result.success;
            measure(r,"validate_ms",[&] {
                double maxVel = 0, maxAcc = 0;
                for(const auto& sample : result.trajectory) {
                    r.path.push_back(sample.state.head<2>().cast<float>());
                    r.velocity.push_back(sample.state.tail<2>()); r.acceleration.push_back(sample.acceleration);
                    r.trajectoryTime.push_back(sample.time);
                    maxVel = std::max(maxVel,sample.state.tail<2>().cwiseAbs().maxCoeff());
                    maxAcc = std::max(maxAcc,sample.acceleration.cwiseAbs().maxCoeff());
                    if(r.path.size()>1 && astar->lineInObsticle(r.path[r.path.size()-2],r.path.back())) ++r.collisions;
                }
                r.passed = result.success && !r.path.empty() && r.collisions == 0 &&
                    (result.trajectory.back().state.head<2>()-c.end.cast<double>()).norm()<1e-6 &&
                    result.trajectory.back().state.tail<2>().norm()<1e-6 &&
                    maxVel<=astar->config.maxVel+1e-6 && maxAcc<=astar->config.maxAcc+1e-6;
                r.reason = r.passed ? "success" : (result.success ? "validation_failure" : "frontend_failure");
            });
        } else {
            std::unique_ptr<YAstar> astar;
            std::unique_ptr<MinimumSnap> solver;
            MinimumSnap::SolveInput input;
            measure(r,"initialize_ms",[&] {
                solver.reset(new MinimumSnap);
                MinimumSnap::Map grid(map.rows,map.cols);
                for(int y=0;y<map.rows;++y) for(int x=0;x<map.cols;++x) grid(y,x)=map.at<unsigned char>(y,x);
                configure(*solver,input,o,grid);
                if(c.group != "speed") {
                    astar.reset(new YAstar(map.cols,map.rows,o.resolution,0,0));
                    astar->setMap(map.cols,map.rows,const_cast<unsigned char*>(map.ptr<unsigned char>()));
                    astar->setMaskNumLayers(4);
                }
            });
            if(c.group == "speed") {
                if(c.items.size()<2) throw std::runtime_error("invalid_speed_items");
                for(const auto& item : c.items) {
                    if(!inside(item.xy,o,map)) throw std::runtime_error("input_out_of_map");
                    r.front.push_back(item.xy);
                }
                input.setItems(c.items);
                auto times = MinimumSnap::trapezoidalTimeAllocation(r.front,8.f,2.f);
                input.setTimeAllocated(times);
            } else {
                measure(r,"cost_map_ms",[&] { astar->setCostField(1.f,[](float x){return .5f/(.1f+x);}); astar->initCostMap(); });
                measure(r,"search_ms",[&] { r.front = astar->search(c.start,c.end); });
                r.frontendSuccess = r.front.size()>=2 && (r.front.back()-c.end).norm()<=.1f;
                if(!r.frontendSuccess) {
                    r.reason = "frontend_failure"; r.total=milliseconds(begin); return r;
                }
                measure(r,"simplify_ms",[&] {
                    r.front=astar->simplifyPath(r.front,o.simplifyThreshold);
                    r.front=astar->simplifyPathHypot(r.front);
                });
                input.setPath(r.front); input.setInitVel(Point::Zero());
            }
            input.setBackend(c.backend=="Close" ? MinimumSnap::Backend::Close : MinimumSnap::Backend::OSQPCorridor);
            r.backendAttempted = true;
            measure(r,"optimize_ms",[&] { r.solve=solver->solve(input); });
            measure(r,"validate_ms",[&] {
                r.path=r.solve.path;
                bool valid = !r.path.empty() && (r.path.front()-c.start).norm()<=.1f && (r.path.back()-c.end).norm()<=.1f;
                for(size_t i=0;i<r.path.size();++i) {
                    valid = inside(r.path[i],o,map) && valid;
                    if(i && solver->lineInObsticle(r.path[i-1],r.path[i])) ++r.collisions;
                }
                if(c.group=="speed") {
                    // Preserve the original example's position, sampled direction and corridor checks.
                    for(const auto& item : c.items) if(item.useVxvy) {
                        size_t index=r.path.size(); double distance=std::numeric_limits<double>::infinity();
                        for(size_t i=0;i<r.path.size();++i) {
                            distance=std::min(distance,static_cast<double>((r.path[i]-item.xy).norm()));
                            if(index==r.path.size() && (r.path[i]-item.xy).norm()<1e-4f) index=i;
                        }
                        valid = valid && distance<1e-4 && index+1<r.path.size();
                        if(index+1<r.path.size()) {
                            Point tangent=r.path[index+1]-r.path[index];
                            if(tangent.norm()==0 || item.vxvy.norm()==0) valid=false;
                            else {
                                double cosine=std::max(-1.,std::min(1.,static_cast<double>(tangent.normalized().dot(item.vxvy.normalized()))));
                                valid=valid && std::acos(cosine)*180/std::acos(-1.)<5.;
                            }
                        }
                        if(c.backend!="Close") {
                            valid=valid && std::find(r.solve.corridor.begin(),r.solve.corridor.end(),item.corridor)!=r.solve.corridor.end();
                        }
                    }
                }
                r.passed=r.solve.success && valid && r.collisions==0;
                r.reason = r.passed ? "success" : (!r.solve.success ? r.solve.exitReason : "validation_failure");
            });
        }
    } catch(const std::exception& e) {
        r.reason = std::string("exception:")+e.what();
    }
    r.total=milliseconds(begin);
    return r;
}
void writePoints(cv::FileStorage& fs, const std::string& name, const std::vector<Point>& points) {
    fs << name << "[";
    for(const auto& p : points) fs << "{" << "x" << static_cast<double>(p.x()) << "y" << static_cast<double>(p.y()) << "}";
    fs << "]";
}
void saveRecord(const std::string& directory, const Record& r) {
    const auto path=directory+"/cases/"+std::to_string(r.input.id)+".json";
    auto fs=writer(path+".tmp");
    fs << "case_id" << r.input.id << "group" << r.input.group << "backend" << r.input.backend
       << "passed" << static_cast<int>(r.passed) << "reason" << r.reason << "total_ms" << r.total
       << "frontend_success" << static_cast<int>(r.frontendSuccess)
       << "backend_attempted" << static_cast<int>(r.backendAttempted)
       << "backend_success" << static_cast<int>(r.solve.success)
       << "iterations" << r.solve.iter << "collision_lines" << r.collisions
       << "collision_segments" << r.solve.collisionSegments
       << "iteration_limit_reached" << static_cast<int>(r.solve.iterationLimitReached)
       << "time_limit_reached" << static_cast<int>(r.solve.timeLimitReached)
       << "backend_failure" << r.solve.backendFailure << "osqp_axis" << r.solve.osqpAxis
       << "osqp_status" << r.solve.osqpStatus << "osqp_error" << r.solve.osqpError
       << "timings" << "{";
    for(const auto& timing : r.timings) fs << timing.first << timing.second;
    fs << "}";
    writePoints(fs,"front",r.front); writePoints(fs,"path",r.path);
    fs << "corridors" << "[";
    for(const auto& box : r.solve.corridor) { fs << "["; for(float v:box) fs << v; fs << "]"; }
    fs << "]" << "motion" << "[";
    for(size_t i=0;i<r.velocity.size();++i) fs << "{" << "vx" << r.velocity[i].x() << "vy" << r.velocity[i].y()
        << "ax" << r.acceleration[i].x() << "ay" << r.acceleration[i].y() << "t" << r.trajectoryTime[i] << "}";
    fs << "]";
    fs.release();
    if(std::rename((path+".tmp").c_str(),path.c_str())!=0)
        throw std::runtime_error("Cannot finalize case record: "+path);
}
bool display(const Record& r, const Options& o, const cv::Mat& map) {
    cv::Mat image; cv::cvtColor(map,image,cv::COLOR_GRAY2BGR);
    auto pixel=[&](const Point& p){return cv::Point(cvRound(p.x()/o.resolution),cvRound(p.y()/o.resolution));};
    auto line=[&](const std::vector<Point>& points,cv::Scalar color) {
        std::vector<cv::Point> pixels;
        for(const auto& p:points) if(p.allFinite()) pixels.push_back(pixel(p));
        if(pixels.size()>1) cv::polylines(image,pixels,false,color,1,cv::LINE_AA);
    };
    for(const auto& box:r.solve.corridor) {
        if(!std::all_of(box.begin(),box.end(),[](float v){return std::isfinite(v);})) continue;
        cv::rectangle(image,pixel(Point(box[0],box[1])),pixel(Point(box[2],box[3])),cv::Scalar(255,0,0),1);
    }
    line(r.front,cv::Scalar(0,128,0)); line(r.path,cv::Scalar(0,0,192));
    for(const auto& item:r.input.items) if(item.useVxvy && item.vxvy.norm()>0)
        cv::arrowedLine(image,pixel(item.xy),pixel(item.xy+item.vxvy.normalized()),cv::Scalar(0,160,160),1,cv::LINE_AA);
    for(size_t i=0;i<r.velocity.size();i+=100) if(r.velocity[i].norm()>1e-6)
        cv::arrowedLine(image,pixel(r.path[i]),pixel(r.path[i]+(.5*r.velocity[i].normalized()).cast<float>()),cv::Scalar(0,160,160),1,cv::LINE_AA);
    cv::circle(image,pixel(r.input.start),3,cv::Scalar(0,200,0),-1);
    cv::circle(image,pixel(r.input.end),3,cv::Scalar(255,0,0),-1);
    const double scale=std::min(2.,std::min(1200./image.cols,800./image.rows));
    cv::resize(image,image,cv::Size(),scale,scale,cv::INTER_NEAREST);
    cv::putText(image,"case "+std::to_string(r.input.id)+" "+r.reason+" "+std::to_string(r.total)+" ms",
        cv::Point(8,20),cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(0,0,0),1,cv::LINE_AA);
    cv::imshow("path",image);
    // GTK's WND_PROP_VISIBLE can return -1 even for an open window.
    while(true) {
        int key=cv::waitKey(50);
        if(key>=0) return (key&255)!=27 && (key&255)!='q' && (key&255)!='Q';
        try { if(cv::getWindowImageRect("path").width<=0) return false; }
        catch(const cv::Exception&) { return false; }
    }
}
void csvHeader(std::ostream& out) {
    out << "case_id,group,backend,passed,reason,total_ms,frontend_success,backend_attempted,backend_success,iterations,collision_lines,collision_segments,iteration_limit_reached,time_limit_reached,backend_failure,osqp_axis,osqp_status,osqp_error";
    for(const auto& s:stages) out<<','<<s;
    out<<'\n';
}
std::string quote(const std::string& value) {
    std::string out="\""; for(char c:value){ if(c=='\"') out+='\"'; out+=c; } return out+'\"';
}
void csvRecord(std::ostream& out,const Record& r) {
    out<<std::setprecision(17)<<r.input.id<<','<<r.input.group<<','<<r.input.backend<<','<<r.passed<<','<<quote(r.reason)<<','<<r.total
       <<','<<r.frontendSuccess<<','<<r.backendAttempted<<','<<r.solve.success<<','<<r.solve.iter<<','<<r.collisions
       <<','<<r.solve.collisionSegments<<','<<r.solve.iterationLimitReached<<','<<r.solve.timeLimitReached
       <<','<<r.solve.backendFailure<<','<<r.solve.osqpAxis<<','<<r.solve.osqpStatus<<','<<r.solve.osqpError;
    for(const auto& s:stages){ out<<','; auto it=r.timings.find(s); if(it!=r.timings.end()) out<<it->second; }
    out<<'\n'<<std::flush;
}
void help() {
    std::cout<<"play [--mode demo|batch] [--map IMAGE] [--resolution M_PER_PIXEL]\n"
        "     [--suite existing|random] [--count 1000] [--seed UINT32]\n"
        "     [--output DIRECTORY] [--inputs SAVED_INPUTS_JSON] [--case-id ID]\n"
        "     [--max-iterations 6] [--time-limit 10] [--max-speed 3] [--max-acc 1]\n"
        "     [--order 6] [--max-derivative 3] [--corridor-range 2.5]\n"
        "No arguments: original interactive demos; Q/Esc quits. All times are ms.\n";
}
}
int run(int argc,char** argv) {
    try {
        Options o;
        for(int i=1;i<argc;++i) {
            std::string key=argv[i];
            if(key=="--help" || key=="-h") {help();return 0;}
            if(key=="--inputs") {
                if(i+1>=argc) throw std::runtime_error("Missing --inputs value");
                o.inputs=argv[++i]; cv::FileStorage fs(o.inputs,cv::FileStorage::READ);
                if(!fs.isOpened()) throw std::runtime_error("Cannot read saved inputs");
                readConfig(fs,o);
            }
        }
        bool explicitMap=false,explicitSuite=false;
        for(int i=1;i<argc;++i) {
            std::string key=argv[i];
            if(i+1>=argc) throw std::runtime_error("Missing value for "+key);
            std::string value=argv[++i];
            if(key=="--mode") o.mode=value;
            else if(key=="--map") {o.map=value;explicitMap=true;}
            else if(key=="--resolution") o.resolution=std::stod(value);
            else if(key=="--suite") {o.suite=value;explicitSuite=true;}
            else if(key=="--output") o.output=value;
            else if(key=="--inputs") o.inputs=value;
            else if(key=="--count") o.count=std::stoi(value);
            else if(key=="--seed") {
                if(value.empty() || value.front()=='-' || value.find_first_not_of("0123456789")!=std::string::npos || std::stoull(value)>UINT32_MAX)
                    throw std::runtime_error("Seed must be a uint32");
                o.seed=static_cast<uint32_t>(std::stoull(value));
            }
            else if(key=="--case-id") {
                std::stringstream stream(value); std::string id;
                while(std::getline(stream,id,',')) o.ids.insert(std::stoi(id));
            }
            else if(key=="--max-iterations") o.maxIterations=std::stoi(value);
            else if(key=="--time-limit") o.timeLimit=std::stod(value);
            else if(key=="--max-speed") o.maxSpeed=std::stod(value);
            else if(key=="--max-acc") o.maxAcc=std::stod(value);
            else if(key=="--order") o.order=std::stoi(value);
            else if(key=="--max-derivative") o.maxDerivative=std::stoi(value);
            else if(key=="--corridor-range") o.corridorRange=std::stod(value);
            else throw std::runtime_error("Unknown argument: "+key);
        }
        if(explicitMap && !explicitSuite && o.inputs.empty()) o.suite="random";
        auto positive=[](double v){return std::isfinite(v)&&v>0;};
        if((o.mode!="demo"&&o.mode!="batch") || (o.suite!="existing"&&o.suite!="random") ||
           o.count<0 || !positive(o.resolution) || !positive(o.timeLimit) || !positive(o.maxSpeed) || !positive(o.maxAcc) ||
           o.maxIterations<1 || o.order<3 || o.maxDerivative<2 || !std::isfinite(o.corridorRange) || o.corridorRange<0)
            throw std::runtime_error("Invalid benchmark parameters");
        cv::Mat map=cv::imread(o.map,cv::IMREAD_GRAYSCALE);
        if(map.empty()) throw std::runtime_error("Cannot read map: "+o.map);
        std::vector<Case> cases;
        if(o.inputs.empty()) cases=generate(o,map);
        else {cv::FileStorage fs(o.inputs,cv::FileStorage::READ);cases=readCases(fs);}
        if(cases.empty()) throw std::runtime_error("No test cases selected");
        std::set<int> available;
        for(const auto& c:cases) {
            if(c.id<1 || !available.insert(c.id).second) throw std::runtime_error("Invalid or duplicate case ID");
            if(!inside(c.start,o,map)||!inside(c.end,o,map)) throw std::runtime_error("Case "+std::to_string(c.id)+" out of map; use --suite random for a different map");
            for(const auto& item:c.items) if(!inside(item.xy,o,map)) throw std::runtime_error("Speed path out of map");
        }
        for(int id:o.ids) if(!available.count(id)) throw std::runtime_error("Unknown case ID: "+std::to_string(id));
        std::cout<<"seed="<<o.seed<<" map="<<o.map<<" size="<<map.cols<<'x'<<map.rows<<" resolution="<<o.resolution<<" suite="<<o.suite<<std::endl;
        std::ofstream csv;
        if(!o.output.empty()) {
            mkdirs(o.output+"/cases");
            std::ifstream previous(o.output+"/inputs.json");
            if(previous.good()) throw std::runtime_error("Output already contains a run; choose a new directory");
            writeInputs(o.output+"/inputs.json",o,cases,map);
            csv.open(o.output+"/results.csv"); if(!csv) throw std::runtime_error("Cannot write results.csv"); csvHeader(csv);
        }
        size_t completed=0,passed=0; bool stopped=false,exception=false;
        std::map<std::string,std::vector<double>> timings;
        for(const auto& c:cases) {
            if(!o.ids.empty()&&!o.ids.count(c.id)) continue;
            Record r=execute(c,o,map); ++completed; passed+=r.passed;
            exception=exception || r.reason.find("exception:")==0;
            for(const auto& t:r.timings) timings[t.first].push_back(t.second);
            timings["total_ms"].push_back(r.total);
            if(csv.is_open()) {saveRecord(o.output,r);csvRecord(csv,r);}
            std::cout<<"case="<<c.id<<" group="<<c.group<<" backend="<<c.backend<<" passed="<<r.passed
                <<" reason="<<r.reason<<" total_ms="<<r.total<<" iter="<<r.solve.iter
                <<" collision_lines="<<r.collisions<<" collision_segments="<<r.solve.collisionSegments
                <<" iteration_limit="<<r.solve.iterationLimitReached<<" time_limit="<<r.solve.timeLimitReached
                <<" backend_failure="<<r.solve.backendFailure<<" osqp_status="<<r.solve.osqpStatus<<std::endl;
            if(o.mode=="demo"&&!display(r,o,map)) {stopped=true;break;}
        }
        if(o.mode=="demo") cv::destroyAllWindows();
        std::cout<<"completed="<<completed<<" pass="<<passed<<" fail="<<completed-passed<<" rate="<<(completed?100.*passed/completed:0)<<"%"<<std::endl;
        for(const auto& t:timings) {
            double sum=0;for(double value:t.second)sum+=value;
            std::cout<<t.first<<" count="<<t.second.size()<<" sum="<<sum<<" mean="<<sum/t.second.size()
                <<" max="<<*std::max_element(t.second.begin(),t.second.end())<<std::endl;
        }
        if(!o.output.empty()) {auto fs=writer(o.output+"/completion.json");fs<<"completed"<<static_cast<int>(completed)<<"stopped"<<static_cast<int>(stopped)<<"exception"<<static_cast<int>(exception);}
        return exception?2:0;
    } catch(const std::exception& e) {std::cerr<<"benchmark error: "<<e.what()<<std::endl;return 2;}
}
}
