#include "item_stack.hpp"

namespace godot {
ItemStack::ItemStack(Ref<ItemBlueprint> bp, int count, Dictionary&& data)
    : blueprint(bp), count(count), instance_data(std::move(data)) {}

auto ItemStack::is_valid() const -> bool { return blueprint.is_valid() && count > 0; }

auto ItemStack::can_stack_with(const ItemStack& other) const -> bool {
    return blueprint == other.blueprint && instance_data == other.instance_data;
}

auto ItemStack::get_max_stack_size() const -> int { return blueprint->get_max_stack_size(); }

// Returns the part that couldn't be merged (or an invalid stack if fully merged)
auto ItemStack::merge_from(ItemStack& other_stack) -> ItemStack {
    if (!can_stack_with(other_stack)) {
        return ItemStack::invalid();
    }
    int amount_to_merge = std::min(other_stack.count, get_max_stack_size() - other_stack.count);
    count += amount_to_merge;
    other_stack.count -= amount_to_merge;
    return other_stack;
}

// Returns a new stack with the split amount, reduces this stack's quantity
auto ItemStack::split(int amount_to_split) -> ItemStack {
    if (amount_to_split >= count) {
        return ItemStack::invalid();
    }
    ItemStack new_stack = *this;
    new_stack.count     = amount_to_split;
    count -= amount_to_split;
    return new_stack;
}

auto ItemStack::operator==(const ItemStack& other) const -> bool {
    return blueprint == other.blueprint && instance_data == other.instance_data;
}

auto ItemStack::invalid() -> const ItemStack& {
    static const ItemStack invalid_stack;
    return invalid_stack;
}
} // namespace godot