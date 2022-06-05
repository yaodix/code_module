#include "quadtree.h"
#include <iostream>
#include <cmath>
#include <cassert>



Quadtree::Quadtree(int left, int top, int width, int height):
    rect_shape_(sf::Vector2f(width, height)), rect_(left, top, width, height) {
    rect_shape_.setPosition(left, top);
    rect_shape_.setOutlineThickness(KQuadtreeOutlineThickness);
    rect_shape_.setOutlineColor(sf::Color::Black);
    rect_shape_.setFillColor(sf::Color(255, 255, 255, 0));
}


bool Quadtree::InsertElement(Ball *b) {
    bool intersects_left = b->GetX() >= GetRect().left - b->GetR();
    bool intersects_right = b->GetX() <= GetRect().left + GetRect().width + b->GetR();
    bool intersects_horizontal = intersects_left && intersects_right;
    bool intersects_top = b->GetY() >= GetRect().top - b->GetR();
    bool intersects_bottom = b->GetY() <= GetRect().top + GetRect().height + b->GetR();
    bool intersects_vertical = intersects_top && intersects_bottom;
    bool intersects = intersects_horizontal && intersects_vertical;
    if(!intersects) {
        return false;
    }
    if(GetSize() < kQuadtreeNodeCapacity) {
        children_.push_back(b);
        return true;
    }
    if(NW_ == nullptr) {
        Subdivide(this);
    }
    bool insert_NW = NW_->InsertElement(b);
    bool insert_NE = NE_->InsertElement(b);
    bool insert_SW = SW_->InsertElement(b);
    bool insert_SE = SE_->InsertElement(b);
    if(insert_NW || insert_NE || insert_SW || insert_SE) {
        return true;
    }
    std::cout << "In Insert element(), this should never happen." << std::endl;
    //assert(false);
    return false;
}


void Quadtree::Subdivide(Quadtree *root) {
    int new_width = GetRect().width / 2;
    int new_height = GetRect().height / 2;
    NW_ = new Quadtree(GetRect().left, GetRect().top, new_width, new_height);
    NE_ = new Quadtree(GetRect().left + new_width, GetRect().top, new_width, new_height);
    SW_ = new Quadtree(GetRect().left, GetRect().top + new_height, new_width, new_height);
    SE_ = new Quadtree(GetRect().left + new_width, GetRect().top + new_height, new_width, new_height);
    for (auto &node: root->children_) {
        NW_->InsertElement(node);
        NE_->InsertElement(node);
        SW_->InsertElement(node);
        SE_->InsertElement(node);
    }
}


void Quadtree::QtreeCheckCollisions(int &cnt) {
    Quadtree *NW = GetNW();
    Quadtree *NE = GetNE();
    Quadtree *SW = GetSW();
    Quadtree *SE = GetSE();

    if (NW != nullptr) {
        NW->QtreeCheckCollisions(cnt);
        NE->QtreeCheckCollisions(cnt);
        SW->QtreeCheckCollisions(cnt);
        SE->QtreeCheckCollisions(cnt);
        return;
    }

    std::vector<Ball *> children_vec = GetChildren();
    for (auto node_one: children_vec) {
        for (auto node_two: children_vec) {
            if (node_one == node_two) {
                continue;
            }
            node_one->CheckCollisionWithBall(node_two);
            ++cnt;
        }
    }
}


void Quadtree::QtreeFreeMemory() {
    if(GetNW()){
        GetNW()->QtreeFreeMemory();
        GetNE()->QtreeFreeMemory();
        GetSW()->QtreeFreeMemory();
        GetSE()->QtreeFreeMemory();
    }
    delete this;
}
