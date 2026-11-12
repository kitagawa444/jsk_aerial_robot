#pragma once
#include <bee/cooperative_grasp/types.h>

namespace bee::grasp
{
// Contact representatives in object coordinates. Independent of world height,
// robot count, and the initial differences between attachment positions.
class ContactGeometry
{
public:
  void reset() { reference_.clear(); }
  bool initialized() const { return !reference_.empty(); }
  void capture(const GraspState& state)
  {
    reference_.clear();
    for (const auto& contact : state.contacts)
      reference_.push_back(state.input.object.orientation.conjugate() *
                           (contact.position - state.input.object.position));
  }
  std::vector<Vec3> errors(const GraspState& state) const
  {
    if (!initialized() || reference_.size() != state.contacts.size())
      throw std::logic_error("contact geometry reference unavailable");
    std::vector<Vec3> errors;
    for (std::size_t i = 0; i < reference_.size(); ++i)
      errors.push_back(state.input.object.orientation.conjugate() *
                       (state.contacts[i].position - state.input.object.position) - reference_[i]);
    return errors;
  }
private:
  std::vector<Vec3> reference_;
};
}  // namespace bee::grasp
