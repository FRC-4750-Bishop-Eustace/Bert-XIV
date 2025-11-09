# 🧭 BertLib Style Guide

This document defines the official formatting and naming rules for this project. Every contributor must follow these conventions to ensure consistent, readable, and maintainable code.

## 🧱 General Conventions

- **Tabs vs Spaces:** Always use **spaces** (no tabs)
- **Indentation:** 4 spaces per indent level
- **Line Endings:** Use `LF` (`\n`)
- **Charset:** UTF-8
- **Maximum Line Length:** 120 characters
- **Trailing Whitespace:** None
- **Final Newline:** Required

> 🧩 These rules are automatically enforced by the provided [`.editorconfig`](https://github.com/Rohan-Bharatia/BertLib/blob/main/.editorconfig) file.

## 📁 File & Directory Naming Conventions

|**Type**|**Convention**|**Example**|
|-|-|-|
|**Directories/Folders**|Lowercase Snake Case|`hardware/`|
|**Python Files**|Lowercase Snake Case With `.py` Extension|`robot.py`|

- The main class or struct inside a file should **match the file name**.
- Directory underscores should only be used **when absolutely necessary**, but nested directories is preferred.

## 🧾 File Structure

Python file example:
```py
# MIT License
#
# Copyright (c) 2025 Beʳᵗ FRC Team 4750
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import ...

...

```

## 🧠 Naming Conventions

|**Element**|**Style**|**Example**|
|-|-|-|
|**Classes/Enums**|Pascal Case|`Robot`, `RobotContainer`|
|**Enum Values**|Pascal Case prefixed with `k`|`kNone`, `kCTRETalonFX`|
|**Methods**|Pascal Case|`SetSpeed(self, speed)`, `Reset(self)`|
|**Function Parameters**|Camel case|`value`, `deadzoneStrength`|
|**Member Variables**|Camel Case|`fps`, `motor`|
|**Constants**|Uppercase Snake Case|`AXIS_LEFT_X = ...`, `CPU_MAX_TEMPERATURE_CELSIUS = ...`|

> Define functions to use `(self)` for functions within classes.

## 📚 Includes

Always define all imports at the top of the python file after the license

## 🏗️ Class & Struct Layout

### 🔸 Class Member Order

1. Member variables
2. Defined constructors
3. Abstract methods
4. Defined methods

---

### 🧩 Enum Rules

- All `enum` types must start with: `kNone,`.
- Always include a trailing comma on the last element.

## 🧭 Best Practices

- Always use **OOP principles**: every file should generally center around a main class or structure.
- Avoid implicit conversions: use explicit casting where appropriate.
- Never suppress compiler warnings without justification.

## 🚫 Forbidden

- Tabs for indentation
- Enum definitions without `kNone,`
- Trailing whitespace
- etc.

## ❓ FAQ

- For answers to questions about this document, contact any contributors.
