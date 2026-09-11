#pragma once
#include "../Components.h"
#include "../ReportText.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <queue>

namespace crash::cases::vehicle_startup::connectivity::test {
struct InputRelation { Kind kind; Role role; std::uint64_t id,part; std::vector<std::uint32_t> slots; };
struct Fixture {
    std::vector<tl::fea::NodalDomainNode> nodes;
    std::vector<InputRelation> relations;
    Fixture() {
        const std::uint64_t ids[]{900,40,700,8,500,60,70,90,1000,2,3,4,120,130,140,150,160,170,1};
        for (std::size_t i=0;i<std::size(ids);++i) nodes.push_back({ids[i],{double(i),0,0}});
        nodes[8].position = nodes[3].position; // Distinct NIDs at exactly equal coordinates.
        relations = {
            {Kind::Qeph,Role::Constitutive,7001,101,{0,1,2,3}},
            {Kind::T3,Role::Constitutive,7002,101,{4,5,6}},
            {Kind::Qeph,Role::RigidSkin,7003,303,{7,8,9,10}},
            {Kind::Type13,Role::Constitutive,7004,404,{2,4}}, // N3 would be8; it is not a support.
            {Kind::Type25,Role::Constitutive,7004,0,{6,11}}, // Distinct WID namespace.
            {Kind::Solid6z,Role::Constitutive,7005,505,{12,13,14,15,16,17}},
            {Kind::PointMass,Role::CoefficientOnly,7004,0,{9}}, // Independent mass-card namespace.
            {Kind::PointMass,Role::CoefficientOnly,7006,0,{18}},
            {Kind::PartRoot,Role::Constraint,303,303,{7,8,9,10}},
            {Kind::PlainGroup,Role::Constraint,201,0,{11,12}},
            {Kind::Cin,Role::Constraint,7003,303,{5,7,8,9,9}}};
    }
    tl::util::ConstView<tl::fea::NodalDomainNode> Nodes() const { return {nodes.data(),nodes.size()}; }
    Data Build(bool reverse=false) const {
        Data data;
        data.element_label.resize(nodes.size(),999); data.transfer_label.resize(nodes.size(),999);
        data.node_roles.resize(nodes.size(),0xff);
        for (std::size_t order=0;order<relations.size();++order) {
            const auto i = reverse ? relations.size()-1-order : order;
            const auto& row = relations[i];
            data.relations.push_back({row.id,row.part,static_cast<std::uint32_t>(i),
                static_cast<std::uint32_t>(data.slots.size()),static_cast<std::uint32_t>(row.slots.size()),
                row.kind,row.role,static_cast<std::uint16_t>(row.role==Role::RigidSkin ? 0 : UINT16_MAX)});
            data.slots.insert(data.slots.end(),row.slots.begin(),row.slots.end());
        }
        return data;
    }
};
// Independent explicit adjacency/BFS oracle. This does not call Components,
// classify production roles, or use the production union traversal order.
inline std::vector<std::uint64_t> Bfs(const Fixture& source, bool transfer) {
    std::vector<std::vector<std::uint32_t>> edges{{0,1,2,3},{4,5,6},{2,4},{6,11},{12,13,14,15,16,17}};
    if (transfer) { edges.push_back({7,8,9,10}); edges.push_back({11,12}); edges.push_back({5,7,8,9,9}); }
    std::vector<std::vector<std::uint32_t>> adjacent(source.nodes.size());
    for (const auto& edge : edges) for (auto a : edge) for (auto b : edge)
        if (a!=b) adjacent[a].push_back(b);
    std::vector<std::uint64_t> labels(source.nodes.size());
    std::vector<bool> visited(source.nodes.size());
    for (std::size_t first=0;first<source.nodes.size();++first) {
        if (visited[first]) continue;
        std::queue<std::uint32_t> pending;
        std::vector<std::uint32_t> component;
        std::uint64_t minimum=UINT64_MAX;
        pending.push(first); visited[first]=true;
        while (!pending.empty()) {
            const auto node=pending.front(); pending.pop(); component.push_back(node);
            minimum=std::min(minimum,source.nodes[node].source_id);
            for (auto other : adjacent[node]) if (!visited[other]) { visited[other]=true; pending.push(other); }
        }
        for (auto node : component) labels[node]=minimum;
    }
    return labels;
}
} // namespace crash::cases::vehicle_startup::connectivity::test
