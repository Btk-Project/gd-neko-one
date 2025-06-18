#pragma once

#include "item_blueprint.hpp"        // Assuming ItemBlueprint is defined
#include <godot_cpp/classes/ref.hpp> // For Ref<ItemBlueprint>
#include <godot_cpp/core/object.hpp> // If not RefCounted, can be simple struct/class
#include <godot_cpp/variant/dictionary.hpp>

namespace godot {

// Forward declaration if ItemBlueprint is complex
// class ItemBlueprint;

class ItemStack { // Could be a struct too if all public
public:
    Ref<ItemBlueprint> blueprint;
    int count = 0;
    Dictionary instance_data; // For durability, enchantments, etc.

    ItemStack() = default;
    explicit ItemStack(Ref<ItemBlueprint> p_bp, int p_qty = 1, Dictionary&& p_data = Dictionary());

    auto is_valid() const -> bool;
    auto can_stack_with(const ItemStack& other) const -> bool;
    auto get_max_stack_size() const -> int;

    // Returns the part that couldn't be merged (or an invalid stack if fully merged)
    auto merge_from(ItemStack& other_stack) -> ItemStack;
    // Returns a new stack with the split amount, reduces this stack's quantity
    auto split(int amount_to_split) -> ItemStack;

    auto operator==(const ItemStack& other) const -> bool;

    static auto invalid() -> const ItemStack&;
};
} // namespace godot