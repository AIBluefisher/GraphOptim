// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

// BSD 3-Clause License

// Copyright (c) 2021, Chenyu
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef UTIL_MAP_UTIL_H_
#define UTIL_MAP_UTIL_H_

#include <glog/logging.h>

#include <iterator>
#include <utility>

namespace gopt {
// Perform a lookup in a map or hash_map, assuming that the key exists.
// Crash if it does not.
//
// This is intended as a replacement for operator[] as an rvalue (for reading)
// when the key is guaranteed to exist.
//
// operator[] is discouraged for several reasons:
//  * It has a side-effect of inserting missing keys
//  * It is not thread-safe (even when it is not inserting, it can still
//      choose to resize the underlying storage)
//  * It invalidates iterators (when it chooses to resize)
//  * It default constructs a value object even if it doesn't need to
template <class Collection>
typename Collection::value_type::second_type& FindOrDie(
    Collection& collection,  // NOLINT
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  CHECK(it != collection.end()) << "Map key not found: " << key;
  return it->second;
}

// Same as above but returns a const reference.
template <class Collection>
const typename Collection::value_type::second_type& FindOrDie(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  CHECK(it != collection.end()) << "Map key not found: " << key;
  return it->second;
}

// Same as above but does not print the key.
template <class Collection>
typename Collection::value_type::second_type& FindOrDieNoPrint(
    Collection& collection,  // NOLINT
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  CHECK(it != collection.end());
  return it->second;
}

// Same as above but returns a const reference.
template <class Collection>
const typename Collection::value_type::second_type& FindOrDieNoPrint(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  CHECK(it != collection.end());
  return it->second;
}

// Perform a lookup in a map or hash_map.
// If the key is present in the map then the value associated with that
// key is returned, otherwise the value passed as a default is returned.
template <class Collection>
const typename Collection::value_type::second_type& FindWithDefault(
    const Collection& collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return value;
  }
  return it->second;
}

// Insert a new key and value into a map or hash_map.
// If the key is not present in the map the key and value are
// inserted, otherwise nothing happens. True indicates that an insert
// took place, false indicates the key was already present.
template <class Collection>
bool InsertIfNotPresent(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& value) {
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(key, value));
  return ret.second;
}

// Insert a new value into a set or hash_set. If the value is not present in
// the set then it is inserted, otherwise nothing happens. True indicates that
// an insert took place, false indicates the value was already present.
template <class Collection>
bool InsertIfNotPresent(Collection* const collection,
                        const typename Collection::value_type& value) {
  std::pair<typename Collection::iterator, bool> ret =
      collection->insert(typename Collection::value_type(value));
  return ret.second;
}

// Perform a lookup in a map or hash_map.
// Same as above but the returned pointer is not const and can be used to change
// the stored value.
template <class Collection>
typename Collection::value_type::second_type* FindOrNull(
    Collection& collection,  // NOLINT
    const typename Collection::value_type::first_type& key) {
  typename Collection::iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}

// Perform a lookup in a map or hash_map.
// Same as above but the returned pointer is not const and can be used to change
// the stored value.
template <class Collection>
const typename Collection::value_type::second_type* FindOrNull(
    const Collection& collection,
    const typename Collection::value_type::first_type& key) {
  typename Collection::const_iterator it = collection.find(key);
  if (it == collection.end()) {
    return 0;
  }
  return &it->second;
}

// Test to see if a set, map, hash_set or hash_map contains a particular key.
// Returns true if the key is in the collection.
template <class Collection, class Key>
bool ContainsKey(const Collection& collection, const Key& key) {
  typename Collection::const_iterator it = collection.find(key);
  return it != collection.end();
}

// Inserts a new key/value into a map or hash_map.
// Dies if the key is already present.
template <class Collection>
void InsertOrDie(Collection* const collection,
                 const typename Collection::value_type::first_type& key,
                 const typename Collection::value_type::second_type& data) {
  typedef typename Collection::value_type value_type;
  CHECK(collection->insert(value_type(key, data)).second)
      << "duplicate key: " << key;
}

template <class Collection>
void InsertOrDieNoPrint(
    Collection* const collection,
    const typename Collection::value_type::first_type& key,
    const typename Collection::value_type::second_type& data) {
  typedef typename Collection::value_type value_type;
  CHECK(collection->insert(value_type(key, data)).second);
}

}  // namespace gopt

#endif  // UTIL_MAP_UTIL_H_
