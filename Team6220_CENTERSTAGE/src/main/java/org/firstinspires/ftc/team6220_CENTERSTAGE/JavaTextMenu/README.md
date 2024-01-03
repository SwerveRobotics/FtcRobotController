# java-text-menu v3.1
A flexible, very cool and definitely professionally coded text-based menu predominantly meant for choosing between enums using a controller.
<p>
<i>By valsei!!</i> [https://github.com/valsei/java-text-menu]
<p>
The intended usage is to remove the clutter of 8+ different autonomous programs for each combination of situations and options. Now just choose one with your own custom menu!

# changelog
A summary of important changes; significant updates will recieve a version increase (1.0 -> 1.1), but minor commenting or readme fixes will just be included in the next significant update.
## v3.1
- flipped the `.getResult()` method's parameters to take the return type class first and element name second
- made `.getResult()` throw an error if the given class doesn't match the given element name
## v3.0
- added scrolling, accessible in the constructor: `TextMenu(viewHeight, viewMargin)` otherwise disabled by default
  - `viewHeight` and `viewMargin` must be 0 or greater
  - `viewHeight` determines the number of element rows to show at once
  - `viewMargin` is the number of element rows that must show above/below the hover cursor at all times
  - note that element rows refers to <i>all</i> menu elements, not just hoverable
- added an element render cache so it doesn't remake the strings for every element every loop cycle
- the menu finish button element now shows number of incomplete elements
- can now set custom names to the true/false options in switch elements
## v2.0
- made the project more modular and flexible, separated hoverable elements and text elements
- added better controller input support
## v1.0
- the project is now functional lol
- supports enum elements and double elements

# Setting up a menu
Start by creating a menu and input object, which is used in bridging controller input to the menu.
```java
TextMenu menu = new TextMenu();
MenuInput menuInput = new MenuInput(MenuInput.InputType.CONTROLLER);
```
Determine what setup for options will be necessary.
```java
enum Colors {
    RED,
    BLUE,
    YELLOW,
    GREEN,
}
double minWaitTime = 5.0;
double maxWaitTime = 15.0;
```
Add corresponding menu elements to the menu. Note that hoverable (interactable) menu elements must have an identifier name for retrieving results later.
```java
menu.add(new MenuHeader("Example text!"))
    .add("Menu header shortcut!?")
    .add() // empty line for spacing
    .add("Pick a color:")
    .add("color-picker-1", Colors.class) // enum selector shortcut
    .add()
    .add("Wait time:")
    .add("wait-slider-1", new MenuSlider(minWaitTime, maxWaitTime))
    .add()
    .add("finish-button-1", new MenuFinishedButton())
;
```
## Interacting with a menu
Once done with creating a menu and menu input object, there should be a loop that passes input to the menu until it is complete. The method `menu.isCompleted()` is an easy way to know when to stop, but this may end prematurely. If some elements always have a value (ex. slider) and are therefore always completed, even if they haven't been reached yet the menu will count as completed. This can be dealt with by either having the end loop logic bound to both the completed state and an extra input (ex. B button), or by adding a `MenuFinishedButton` to the end of the menu.
<p>
Example code:

```java
while (!menu.isCompleted()) {
    // get x,y (stick) and select (A) input from controller
    menuInput.update(x, y, select);
    menu.updateWithInput(menuInput);
    // display the updated menu
    for (String line : menu.toListOfStrings() {
        print(line); // but with appropriate printing method
    }
}
```
## Reading from a menu
Example code:
```java
// the first parameter is the type to return as
Colors chosenColor = menu.getResult(Colors.class, "color-picker-1");
double waitTime = menu.getResult(Double.class, "wait-slider-1");
```
```java
switch (menu.getResult(Colors.class, "color-picker-1")) {
    case RED:
    // etc.
}
```