// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4; -*-
#include "RRTPathPlanner.h"
#include "Mobility.h"
#include "RoadmapNode.h"
#include "Roadmap.h"

using namespace PathEngine;

void Roadmap::clear()
{
    nodes_.clear();
    m_nEdges = 0;
}

Roadmap::~Roadmap()
{
    clear();
}

void Roadmap::addEdge(RoadmapNodePtr from, RoadmapNodePtr to)
{
    from->addChild(to);
    to->addParent(from); 
    m_nEdges++;
}

void Roadmap::integrate(RoadmapPtr rdmp)
{
    for (unsigned int i=0; i<nodes_.size(); i++){
        rdmp->addNode(nodes_[i]);
    }
    nodes_.clear();
}

RoadmapNodePtr Roadmap::node(unsigned int index)
{
    if (index >= nodes_.size()) return RoadmapNodePtr();

    return nodes_[index];
}

void Roadmap::findNearestNode(const Configuration& pos,
                              RoadmapNodePtr & node, double &distance)
{
    if (nodes_.size() == 0){
        node = RoadmapNodePtr();
        return;
    }

    node = nodes_[0];
    Mobility *mobility = planner_->getMobility();
    distance = mobility->distance(node->position(), pos);
    
    double d;
    for (unsigned int i=1; i<nodes_.size(); i++) {
        if (!nodes_[i]->isValid()) continue;
        d = mobility->distance(nodes_[i]->position(), pos);
        if (d < distance) {
            distance = d;
            node = nodes_[i];
        }
    }
}

void Roadmap::findNearNodes(const Configuration &pos, double distance,
                            std::vector<RoadmapNodePtr> &nodes)
{
    nodes.clear();
    if (nodes_.size() == 0) {
        nodes.push_back(RoadmapNodePtr());
        return;
    }

    Mobility *mobility = planner_->getMobility();
    
    double d;
    for (unsigned int i=0; i<nodes_.size(); i++) {
        d = mobility->distance(nodes_[i]->position(), pos);
        if (d <= distance) {
            nodes.push_back(nodes_[i]);
        }
    }
}

void Roadmap::findKNearNodes(const Configuration &pos, int k,
                             std::vector<RoadmapNodePtr> &nodes)
{
    nodes.clear();
    if (nodes_.size() == 0) {
        nodes.push_back(RoadmapNodePtr());
        return;
    }

    Mobility *mobility = planner_->getMobility();
    std::vector<Candidate> candidates;
    for (unsigned int i=0; i<nodes_.size(); i++) {
        if (nodes_[i]->isValid()){
            candidates.push_back(Candidate());
            candidates.back().node = nodes_[i];
            candidates.back().dist = mobility->distance(nodes_[i]->position(), pos);
        }
    }

    if (k > candidates.size()) {
        for (size_t i = 0; i < candidates.size(); i++) {
            nodes.push_back(candidates[i].node);
        }
    } else {
        std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end());
        for (int i = 0; i < k; i++) {
            nodes.push_back(candidates[i].node);
        }
    }
}

RoadmapNodePtr Roadmap::lastAddedNode()
{
    if (nodes_.size() == 0) return RoadmapNodePtr();
    return nodes_[nodes_.size()-1];
}

int Roadmap::indexOfNode(RoadmapNodePtr node)
{
    for (unsigned int i=0; i<nodes_.size(); i++){
        if (nodes_[i] == node) return (int)i; 
    }
    return -1;
}

std::vector<RoadmapNodePtr > Roadmap::DFS(RoadmapNodePtr startNode, RoadmapNodePtr goalNode) 
{
    for (unsigned int i=0; i<nodes_.size(); i++) {
        nodes_[i]->visited(false);
    }

    std::vector<RoadmapNodePtr> path;
    startNode->visited(true);
    path.push_back(startNode);
    while (path.size() > 0) {
        RoadmapNodePtr node = path.back();

        if (node == goalNode) {
            break;
        } else {
            RoadmapNodePtr child = RoadmapNodePtr();
            for (unsigned int i=0; i<node->nChildren(); i++) {
                if (!node->child(i)->visited()) {
                    child = node->child(i);
                    break;
                }
            }
            if (child == NULL) {
                path.pop_back();
            } else {
                child->visited(true);
                path.push_back(child);
            }
        }
    }

    return path;
}

void Roadmap::tryConnection(RoadmapNodePtr from, RoadmapNodePtr to, bool tryReverse)
{
    Mobility *mobility = planner_->getMobility();
    if (mobility->isReachable(from->position(), to->position())){
        addEdge(from, to);
        if (tryReverse && mobility->isReversible()) addEdge(to, from);
    }
    if (tryReverse && !mobility->isReversible()){
        if (mobility->isReachable(to->position(), from->position())){
            addEdge(to, from);
        }
    }
}

bool Roadmap::removeEdge(RoadmapNodePtr from, RoadmapNodePtr to)
{
    return from->removeChild(to) && to->removeParent(from);
}
