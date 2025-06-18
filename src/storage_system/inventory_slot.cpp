#include "inventory_slot.hpp"

namespace godot {
InventorySlot::InventorySlot(int p_index) : slot_index(p_index) {}

auto InventorySlot::is_empty() const -> bool { return !item.is_valid() || item.count == 0; }

auto InventorySlot::can_place_item(const ItemStack& p_item_to_place) const -> bool {
    if (is_empty()) {
        return true;
    }
    if (max_quantity_override == -1) {
        if (p_item_to_place.count > item.get_max_stack_size()) {
            return false;
        }
    } else {
        if (p_item_to_place.count > max_quantity_override) {
            return false;
        }
    }
    return true;
}

// Returns true if item was fully placed/swapped, false otherwise
// For set_item, it might return the old item or an error code
auto InventorySlot::set_item_stack(const ItemStack& p_new_item) -> bool {
    if (can_place_item(p_new_item)) {
        if (item.is_valid()) {
            item.count += p_new_item.count;
        } else {
            item = p_new_item;
        }
        return true;
    }
    return false;
}

auto InventorySlot::clear_item_stack() -> ItemStack {
    ItemStack old_item = item;
    item               = ItemStack();
    return old_item;
}

// Direct access (use with caution, especially with threaded containers)
auto InventorySlot::get_item_stack_mut() -> ItemStack& { return item; }

auto InventorySlot::get_item_stack() const -> const ItemStack& { return item; }

auto InventorySlot::invalid() -> const InventorySlot& {
    static InventorySlot invalid_slot;
    return invalid_slot;
}

} // namespace godot