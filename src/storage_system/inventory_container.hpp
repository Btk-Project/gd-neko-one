#pragma once

#include "inventory_slot.hpp"
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/object.hpp>            // To inherit from, e.g., RefCounted for Godot integration
#include <godot_cpp/templates/local_vector.hpp> // Godot's dynamic array, similar to std::vector
#include <mutex>                                // For std::mutex

namespace godot {

class InventoryContainer : public RefCounted { // Inherit if managed by Godot's ref counting
    // NOLINTBEGIN
    GDCLASS(InventoryContainer, RefCounted);
    // NOLINTEND

private:
    LocalVector<InventorySlot> slots;
    int capacity;
    mutable std::mutex m_mutex; // `mutable` so const methods can lock for read-only access

protected:
    static void _bind_methods(); // If exposing to GDScript

public:
    // --- Constructor ---
    InventoryContainer();
    InventoryContainer(const InventoryContainer&)               = delete;
    InventoryContainer(InventoryContainer&&)                    = delete;
    auto operator=(InventoryContainer&&) -> InventoryContainer& = delete;
    explicit InventoryContainer(int p_capacity); // Non-GDScript constructor
    void initialize(int p_capacity);             // For GDScript or delayed init

    // --- Non-Thread-Safe Interface (for single-threaded contexts or internal use) ---
    // These methods DO NOT acquire the lock. Caller must ensure safety or use within a locked scope.
    auto nts_resize(int new_capacity) -> Error;
    auto nts_add_item(ItemStack item_to_add) -> ItemStack; // Takes by value to modify and return remainder
    auto nts_remove_item_from_slot(int slot_index, int count_to_remove = -1) -> ItemStack;
    auto nts_set_item_in_slot(int slot_index, const ItemStack& item_to_set) -> bool;
    auto nts_get_item_in_slot(int slot_index) const -> const ItemStack&;  // Returns const ref
    auto nts_get_copy_of_item_in_slot(int slot_index) const -> ItemStack; // Returns a copy
    auto nts_get_slot_mut(int slot_index) -> InventorySlot&;              // DANGEROUS: direct mutable access
    auto nts_get_slot(int slot_index) const -> const InventorySlot&;
    auto nts_get_capacity() const -> int;
    auto nts_get_item_count_in_slot(int slot_index) const -> int;   // Example utility
    auto nts_get_item_quality_in_slot(int slot_index) const -> int; // Example utility

    // --- Thread-Safe Interface ---
    // These methods acquire and release the internal mutex.
    auto ts_resize(int new_capacity) -> Error;
    auto ts_add_item(ItemStack item_to_add) -> ItemStack;
    auto ts_remove_item_from_slot(int slot_index, int quantity_to_remove = -1) -> ItemStack;
    auto ts_set_item_in_slot(int slot_index, const ItemStack& item_to_set) -> bool;
    auto ts_get_copy_of_item_in_slot(int slot_index) const -> ItemStack; // Always returns copy for safety
    auto ts_get_capacity() const -> int; // Read-only, still locks for consistency if resize can happen
    auto ts_get_item_count_in_slot(int slot_index) const -> int;
    auto ts_get_item_quality_in_slot(int slot_index) const -> int;

    // --- Advanced Thread-Safe Operations (e.g., for complex transactions) ---
    // Allows performing multiple non-thread-safe operations under a single lock.
    template <typename Func>
    auto ts_with_lock(Func&& func) -> decltype(func(*this)) { // Pass `this` or a non-ts proxy
        std::lock_guard<std::mutex> lock(m_mutex);
        // Be careful what 'func' can do. It has full access to nts_ methods.
        return func(*this); // Example: func might take InventoryContainer&
    }

    // --- GDScript Callable Methods (would typically be thread-safe versions or specific non-ts ones if called only
    // from main thread) --- Example:
    void gd_initialize(int p_capacity);
};
} // namespace godot