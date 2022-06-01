#include <iostream>

class Actor
{
public:
  Actor(std::string name) : name_(name),currentSlapped_(false) {}
  virtual ~Actor() {}
  virtual void update() = 0;

  void swap()
  {
    // 交换缓冲区
    currentSlapped_ = nextSlapped_;

    // 清空新的“下一个”缓冲区。.
    nextSlapped_ = false;
  }
  std::string name() {return name_; }  // 方便调试  
  void slap()       { 
      nextSlapped_ = true; 
      std::cout << this->name() <<" was slapped"  <<std::endl;
  }
  bool wasSlapped() { 
        return currentSlapped_; 
}

private:
// 当前状态为读准备，下一状态为写准备
  bool currentSlapped_;
  bool nextSlapped_;
  std::string name_;
};

// 具体的角色
class Comedian : public Actor
{
public:
  Comedian(std::string name) : Actor(name) { }
  void face(Actor* actor) { facing_ = actor; }

  virtual void update()
  {
    if (wasSlapped()) {
      facing_->slap();
      // std::cout << this->name() << " slapped, so he slap " << facing_->name()<<std::endl;
    } else {
      // std::cout << this->name() << " was not slapped, so he do nothing "<<std::endl;
    }
  }

private:
  Actor* facing_;
};

class Stage
{
public:
  void add(Actor* actor, int index)
  {
    actors_[index] = actor;
  }

void update()
{
  for (int i = 0; i < NUM_ACTORS; i++)
  {
    actors_[i]->update();
  }

  for (int i = 0; i < NUM_ACTORS; i++)
  {
    actors_[i]->swap();
  }
}
 Actor** actors() {return actors_;}

private:
  static const int NUM_ACTORS = 3;

  Actor* actors_[NUM_ACTORS];
};


int main() {
  Stage stage;

Comedian* harry = new Comedian("harry");
Comedian* baldy = new Comedian("baldy");
Comedian* chump = new Comedian("chump");

harry->face(baldy);
baldy->face(chump);
chump->face(harry);

stage.add(harry, 0);
stage.add(baldy, 1);
stage.add(chump, 2);

harry->slap();

stage.update();
stage.update();
stage.update();

}