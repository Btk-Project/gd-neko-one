#include "inventory_container.hpp"
#include <functional>
#include <godot_cpp/variant/utility_functions.hpp> // For print, etc.
#include <utility>

namespace godot {

void InventoryContainer::_bind_methods() {
    // Bind GDScript callable methods here
    ClassDB::bind_method(D_METHOD("gd_initialize", "capacity"), &InventoryContainer::gd_initialize);
}

InventoryContainer::InventoryContainer() : capacity(0) {
    // Default constructor for Godot
}
InventoryContainer::InventoryContainer(int p_capacity) : capacity(p_capacity) {
    slots.resize(capacity);
    for (int i = 0; i < capacity; ++i) {
        slots[i].slot_index = i;
    }
}
void InventoryContainer::initialize(int p_capacity) {
    // Non-thread-safe initialization, assume called once before threaded access
    capacity = p_capacity;
    slots.resize(capacity);
    for (int i = 0; i < capacity; ++i) {
        slots[i].slot_index = i;
    }
}
void InventoryContainer::gd_initialize(int p_capacity) { // GDScript callable
    std::lock_guard<std::mutex> lock(m_mutex);           // Make it thread-safe if it can be called concurrently
    initialize(p_capacity);
}

auto InventoryContainer::nts_resize(int new_capacity) -> Error {
    if (new_capacity < capacity) {
        return ERR_INVALID_PARAMETER;
    }
    slots.resize(new_capacity);
    for (int i = capacity; i < new_capacity; ++i) {
        slots[i].slot_index = i;
    }
    capacity = new_capacity;
    return OK;
}

auto InventoryContainer::nts_remove_item_from_slot(int slot_index, int count_to_remove) -> ItemStack {
    if (slot_index < 0 || slot_index >= capacity) {
        return {};
    }
    auto& slot = slots[slot_index];
    if (slot.item.is_valid() && count_to_remove <= slot.item.count) {
        ItemStack removed_item = slot.item;
        slot.item.count -= count_to_remove;
        if (slot.item.count == 0) {
            slot.item = {};
        }
        return removed_item;
    }
    return {};
}

auto InventoryContainer::nts_set_item_in_slot(int slot_index, const ItemStack& item_to_set) -> bool {
    if (slot_index < 0 || slot_index >= capacity) {
        return false;
    }
    slots[slot_index].item = item_to_set;
    return true;
}

auto InventoryContainer::nts_get_item_in_slot(int slot_index) const -> const ItemStack& {
    if (slot_index < 0 || slot_index >= capacity) {
        return ItemStack::invalid();
    }
    return slots[slot_index].item;
}

auto InventoryContainer::nts_get_copy_of_item_in_slot(int slot_index) const -> ItemStack {
    if (slot_index < 0 || slot_index >= capacity) {
        return ItemStack::invalid();
    }
    return slots[slot_index].item;
}

auto InventoryContainer::nts_get_slot_mut(int slot_index) -> InventorySlot& {
    if (slot_index < 0 || slot_index >= capacity) {
        throw std::out_of_range("Slot index out of range");
    }
    return slots[slot_index];
}

auto InventoryContainer::nts_get_slot(int slot_index) const -> const InventorySlot& {
    if (slot_index < 0 || slot_index >= capacity) {
        return InventorySlot::invalid();
    }
    return slots[slot_index];
}

auto InventoryContainer::nts_get_capacity() const -> int { return capacity; }

auto InventoryContainer::nts_get_item_count_in_slot(int slot_index) const -> int {
    if (slot_index < 0 || slot_index >= capacity) {
        return 0;
    }
    return slots[slot_index].item.count;
}

auto InventoryContainer::nts_get_item_quality_in_slot(int slot_index) const -> int {
    if (slot_index < 0 || slot_index >= capacity) {
        return 0;
    }
    if (slots[slot_index].item.is_valid() && slots[slot_index].item.blueprint.is_valid()) {
        return slots[slot_index].item.count * slots[slot_index].item.blueprint->get_quality();
    }
    return 0;
}

// --- Non-Thread-Safe (NTS) Method Examples ---
auto InventoryContainer::nts_add_item(ItemStack item_to_add) -> ItemStack {
    if (!item_to_add.is_valid()) {
        return {}; // Return invalid/empty
    }

    // 1. Try stacking with existing (simplified)
    if (item_to_add.blueprint.is_valid()) {
        for (int i = 0; i < capacity; ++i) {
            if (slots[i].can_place_item(item_to_add) && slots[i].item.can_stack_with(item_to_add)) {
                int can_take = slots[i].item.get_max_stack_size() - slots[i].item.count;
                if (can_take > 0) {
                    int actually_take = MIN(can_take, item_to_add.count);
                    slots[i].item.count += actually_take;
                    item_to_add.count -= actually_take;
                    if (item_to_add.count <= 0) {
                        return {}; // All added
                    }
                }
            }
        }
    }
    // 2. Try placing in empty slots (simplified)
    for (int i = 0; i < capacity; ++i) {
        if (slots[i].is_empty() && slots[i].can_place_item(item_to_add)) {
            // Handle splitting if item_to_add.quantity > max_stack_size for this slot
            int place_qty    = item_to_add.count;
            int max_for_slot = item_to_add.blueprint.is_valid() ? item_to_add.blueprint->get_max_stack_size() : 1;
            if (slots[i].max_quantity_override > 0) {
                max_for_slot = MIN(max_for_slot, slots[i].max_quantity_override);
            }

            if (place_qty > max_for_slot) {
                slots[i].item       = item_to_add; // Assigns a copy
                slots[i].item.count = max_for_slot;
                item_to_add.count -= max_for_slot;
            } else {
                slots[i].item = item_to_add;
                return {}; // All added
            }
        }
    }
    return item_to_add; // Return remainder
}

// --- Thread-Safe (TS) Method Examples (Wrappers around NTS) ---
auto InventoryContainer::ts_add_item(ItemStack item_to_add) -> ItemStack {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Potentially emit signals after lock is released if using a queue, or hold lock during emit
    ItemStack remainder = nts_add_item(std::move(item_to_add));
    // if (item_to_add.quantity != remainder.quantity) { /* emit inventory_changed */ }
    return remainder;
}

auto InventoryContainer::ts_get_copy_of_item_in_slot(int slot_index) const -> ItemStack {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Add bounds checking
    if (slot_index < 0 || slot_index >= capacity) {
        return {}; // Return invalid/empty
    }
    return slots[slot_index].item; // Return a copy
}

auto InventoryContainer::ts_get_capacity() const -> int {
    std::lock_guard<std::mutex> lock(m_mutex);
    return capacity;
}

auto InventoryContainer::ts_resize(int new_capacity) -> Error {
    std::lock_guard<std::mutex> lock(m_mutex);
    return nts_resize(new_capacity);
}

auto InventoryContainer::ts_remove_item_from_slot(int slot_index, int quantity_to_remove) -> ItemStack {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Potentially emit signals after lock is released if using a queue, or hold lock during emit
    ItemStack remainder = nts_remove_item_from_slot(slot_index, quantity_to_remove);
    // if (remainder.quantity != 0) { /* emit inventory_changed */ }
    return remainder;
}

auto InventoryContainer::ts_set_item_in_slot(int slot_index, const ItemStack& item_to_set) -> bool {
    std::lock_guard<std::mutex> lock(m_mutex);
    // Potentially emit signals after lock is released if using a queue, or hold lock during emit
    bool success = nts_set_item_in_slot(slot_index, item_to_set);
    // if (success) { /* emit inventory_changed */ }
    return success;
}

auto InventoryContainer::ts_get_item_count_in_slot(int slot_index) const -> int {
    std::lock_guard<std::mutex> lock(m_mutex);
    return nts_get_item_count_in_slot(slot_index);
}

auto InventoryContainer::ts_get_item_quality_in_slot(int slot_index) const -> int {
    std::lock_guard<std::mutex> lock(m_mutex);
    return nts_get_item_quality_in_slot(slot_index);
}

} // namespace godot