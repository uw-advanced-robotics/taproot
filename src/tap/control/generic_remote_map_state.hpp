/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TAPROOT_GENERIC_REMOTE_MAP_STATE_HPP_
#define TAPROOT_GENERIC_REMOTE_MAP_STATE_HPP_

#include <cstdint>
#include <list>

#include "tap/communication/serial/remote.hpp"

namespace tap
{
namespace control
{
class GenericRemoteMapState
{
public:
    /**
     * Updates the state of the GenericRemoteMapState based on the state of the Remote.
     */
    virtual void updateState(tap::communication::serial::Remote &remote);
    /**
     * Initializes the keys to the bit mapped set of keys provided.
     * @note `keys` must be mutally exclusive with any set of `negKeys` already provided.
     */
    virtual void initKeys(uint16_t keys);

    /**
     * Initializes the neg keys to the bit mapped set of neg keys provided.
     * @note `negKeys` must be mutally exclusive with any set of `keys` already provided.
     */
    virtual void initNegKeys(uint16_t negKeys);

    /**
     * @see `initKeys`. Interprets the list and passes that on as a bit mapped set of keys.
     */
    virtual void initKeys(const std::list<tap::communication::serial::Remote::Key> &keySet);

    /**
     * @see `initNegKeys`. Interprets the list and passes that on as a bit mapped set of keys.
     */
    virtual void initNegKeys(const std::list<tap::communication::serial::Remote::Key> &negKeySet);
    /**
     * Checks if `this` is a subset of `other`. `this` is a subset of `other` under the following
     * conditions:
     * - Either `this`'s left switch state is `UNKNOWN` or `this`'s left switch state is equal to
     *   `other`'s left switch state.
     * - Either `this`'s right switch state is `UNKNOWN` or `this`'s right switch state is equal to
     *   `other`'s left switch state.
     * - Either `this`'s left mouse button is not initialized or both `this` and `other`'s left
     *   mouse buttons are both initialized.
     * - Either `this`'s right mouse button is not initialized or both `this` and `other`'s right
     *   mouse buttons are both initialized.
     * - `this`'s key set is a subset of `other`'s key set, i.e. `(this.keySet & other.keySet) ==
     *   this.keySet`.
     *
     * @attention This function does not use neg keys to determine if the map
     *      state is a subset.
     *
     * @param[other] The RemoteMapState to check if `this` is a subset of.
     * @return `true` if `this` RemoteMapState is a subset of the `other` RemoteMapState. See above
     * for description of what it means for a `RemoteMapState` to be a subset of another.
     */
    virtual bool stateSubsetOf(const GenericRemoteMapState &other) const;

    /**
     * @return The negKeys currently being used.
     */
    uint16_t getNegKeys() const { return negKeys; }

    /**
     * @return `true` if the neg key set has been initialized, `false` otherwise.
     */
    virtual bool getNegKeysUsed() const { return negKeys != 0; }

    /**
     * @return the current keys initialized in the `RemoteMapState`.
     */
    uint16_t getKeys() const { return keys; }

    virtual bool getLMouseButton() const { return lMouseButton; }

    bool getRMouseButton() const { return rMouseButton; }

    /**
     * Straight equality.
     *
     * @param[in] rms1 The first GenericRemoteMapState to check equality for.
     * @param[in] rms1 The second GenericRemoteMapState to check equality for.
     */
    bool friend operator==(const GenericRemoteMapState &rms1, const GenericRemoteMapState &rms2)
    {
        return rms1.keys == rms2.keys && rms1.negKeys == rms2.negKeys;
    }

    /**
     * Opposite of operator==.
     */
    bool friend operator!=(const GenericRemoteMapState &rms1, const GenericRemoteMapState &rms2)
    {
        return !(rms1 == rms2);
    }

    virtual ~GenericRemoteMapState() = default;

protected:
    uint16_t keys = 0;

    uint16_t negKeys = 0;  // if certain keys are pressed, the remote map will not do mapping

    bool lMouseButton = false;

    bool rMouseButton = false;
};
}  // namespace control
}  // namespace tap

#endif