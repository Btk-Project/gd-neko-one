#pragma once

#include "item_stack.hpp"
#include <godot_cpp/variant/array.hpp> // For allowed_item_types
#include <godot_cpp/variant/string_name.hpp>

namespace godot {

class InventorySlot {
public:
    ItemStack item; // The actual item stack
    int slot_index = -1;

    // Restrictions
    Array allowed_item_types;       // Array of StringName
    StringName required_equip_slot; // If this is an equipment slot
    int max_quantity_override = -1; // -1 means use item's max_stack_size

    explicit InventorySlot(int p_index = -1);

    auto is_empty() const -> bool;
    auto can_place_item(const ItemStack& p_item_to_place) const -> bool;

    // Returns true if item was fully placed/swapped, false otherwise
    // For set_item, it might return the old item or an error code
    auto set_item_stack(const ItemStack& p_new_item) -> bool; // Replaces current
    auto clear_item_stack() -> ItemStack;                     // Returns the cleared item

    // Direct access (use with caution, especially with threaded containers)
    auto get_item_stack_mut() -> ItemStack&; // Non-const access
    auto get_item_stack() const -> const ItemStack&;

    static auto invalid() -> const InventorySlot&;
};
} // namespace godot