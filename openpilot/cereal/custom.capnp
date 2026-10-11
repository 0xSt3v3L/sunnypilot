using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct SelfdriveStateSP @0x81c2f05a394cf4af {
  mads @0 :Mads;

  # MADS: lateral-only engagement
  struct Mads {
    state @0 :State;
    enabled @1 :Bool;    # MADS engaged, including paused
    active @2 :Bool;     # MADS actuating lateral
    available @3 :Bool;  # MADS enabled for this car (alternativeExperience)

    enum State {
      disabled @0;
      paused @1;
      enabled @2;
      softDisabling @3;
      overriding @4;
    }
  }
}

struct ModelDataSP @0xaedffd8f31e7b55d {
  roadEdge @0 :RoadEdge;

  # road edge lane change controller state, published by modeld
  struct RoadEdge {
    leftDetected @0 :Bool;    # blocks an assisted lane change to the left
    rightDetected @1 :Bool;
    leftProb @2 :Float32;     # 1 - roadEdgeStds, 0 below the controller's min speed
    rightProb @3 :Float32;
    leftClearance @4 :Float32;  # m between the car side and the edge, 0 without road edges
    rightClearance @5 :Float32;
  }
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct CustomReserved10 @0xcb9fd56c7057593a {
}

struct CustomReserved11 @0xc2243c65e0340384 {
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct CustomReserved13 @0xcd96dafb67a082d0 {
}

struct CustomReserved14 @0xb057204d7deadf3f {
}

struct CustomReserved15 @0xbd443b539493bc68 {
}

struct CustomReserved16 @0xfc6241ed8877b611 {
}

struct CustomReserved17 @0xa30662f84033036c {
}

struct CustomReserved18 @0xc86a3d38d13eb3ef {
}

struct CustomReserved19 @0xa4f1eb3323f5f582 {
}
