// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once
#include <tinyxml2.h>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace paparazzi::xml {
inline std::string attr(const tinyxml2::XMLElement* e, const char* key, const char* fallback = "") {
  const char *s = e ? e->Attribute(key) : nullptr;
  return s ? s : fallback;
}
inline void load(tinyxml2::XMLDocument& doc, const std::filesystem::path& path) {
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error(path.string() + ": " + doc.ErrorStr());
}
template<class Visitor>
void visitXml(const tinyxml2::XMLElement* node, const Visitor& callback) {
  if (!node)
    return;
  callback(node);
  for (auto * child = node->FirstChildElement(); child; child = child->NextSiblingElement())
    visitXml(child, callback);
}
}
