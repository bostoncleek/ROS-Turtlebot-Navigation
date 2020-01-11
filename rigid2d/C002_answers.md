## Boston Cleek

1. The default accessibility of member variables in a class is private and public for a struct.

2. Vector2D is a struct because the data is related and it concept of a vector is easier to comprehend (guideline C.1). Transform2D is a class because it has private data members (theta, ctheta ...) (guideline C.8).

3. They are explicit to avoid unintended conversions (guideline C.46). Single-argument constructors should be declared explicit.

4. * Design 1: Use a struct to store the normal vector and a helper functions in the rigid2d namespace to compose the normal vector. Pros: it will be easier to use and does not require any of the current methods within the Transform2D class. Cons: if we wanted to make the normal vector data types private they would need to be in a class.

 * Design 2: Create a normalize class. Pros: The normal vector would behave like a std::vector or array of doubles making it a concrete type. Cons: It does not need to be a class because the data types will likely need to be visible (public), a class is more work and not easily as readable as a struct.

 * Design 3: Create a normal vector struct that inherits from the Vector2D struct. Pros: This builds a relationship between the objects. Cons: Can make the code more confusing to read if handled poorly.

5. I implemented design 1.

6. The member is declared const because it is not intended to modify the object's observable state.
