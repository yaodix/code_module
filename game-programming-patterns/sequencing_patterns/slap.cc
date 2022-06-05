#include <iostream>
using namespace std;

class Actor
{
public:
  Actor(std::string name) : name_(name),slapped_(false) {}

  virtual ~Actor() {}
  virtual void update() = 0;

  void reset()      { slapped_ = false; }
  void slap()       { 
    slapped_ = true;
    std::cout << this->name() <<" was slapped"  <<std::endl;
  }
  bool wasSlapped() { return slapped_; }
  std::string name() {return name_; }  // 方便调试  
private:
  std::string name_;
  bool slapped_;
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
      actors_[i]->reset();
    }
  }

private:
  static const int NUM_ACTORS = 3;

  Actor* actors_[NUM_ACTORS];
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
      std::cout << this->name() << " slapped, so he slap " << facing_->name()<<std::endl;
    } else {
      std::cout << this->name() << "was not slapped, so he do nothing "<<std::endl;
    }
  }

private:
  Actor* facing_;
};

int main() {
  Stage stage;

Comedian* harry = new Comedian("harry");
Comedian* baldy = new Comedian("baldy");
Comedian* chump = new Comedian("chump");

harry->face(baldy);
baldy->face(chump);
chump->face(harry);

stage.add(harry, 2);
stage.add(baldy, 1);
stage.add(chump, 0);

harry->slap();

stage.update();
}