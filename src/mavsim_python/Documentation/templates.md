# Templates

## Module

### Docstring

The module docstring should be located at the start of a module file without any content before it. The `Functions` and `Classes` header can be removed if there are no functions for classes respectively.

```
"""[file name].py: [summary of module]

[detailed explanation of module]

Functions:
  [public function 1 name]: [summary of function]
  [public function 2 name]: [summary of function]
  ...

Classes:
  [public class 1 name]: [summary of class]
  [public class 2 name]: [summary of class]
  ...

Copyright (c) (202?) ....
"""
```

### `__init__.py` Docstring

The `__init__.py` module docstring should be located at the start of a module file without any content before it.

```
"""__init__.py: [summary of module]

Copyright (c) (202?) ...
"""
```

## Class

### Docstring

#### Public

The detailed explanation can be removed if not needed. However, it is highly recommended.

```
"""[summary of class]

[detailed explanation of class]

Attributes:
  [attribute 1 name]: [attribute 1 description]
  [attribute 2 name]: [attribute 2 description]
  ...
"""
```

#### Private

```
"""[summary of class]"""
```

## Function
Summaries should be in impertive tense for all functions.

### Docstring

#### Public

For a public function that returns ```None```, the ```Returns``` section is optional
```
"""[summary of function]

[details of function]

Args:
  [parameter 1 name]: [parameter 1 description]
  [parameter 2 name]: [parameter 2 description]
  ...

Returns:
  [return description]
"""
```

#### Private or getter/setter

```
"""[summary of function]"""
```

Copyright (c) (202?) ... - All rights reserved