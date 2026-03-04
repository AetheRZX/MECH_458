# Fliptable System Architecture

## Overall System Diagram

```mermaid
flowchart TD
    %% Global Graph Settings
    %% "stepAfter" creates orthogonal/manhattan style lines
    linkStyle default interpolate stepAfter stroke:#555,stroke-width:2px;

    %% Class Definitions - Professional Palette
    classDef control fill:#e1f5fe,stroke:#01579b,stroke-width:2px,color:#01579b;
    classDef power fill:#ffebee,stroke:#b71c1c,stroke-width:2px,color:#b71c1c;
    classDef lift fill:#f3e5f5,stroke:#4a148c,stroke-width:2px,color:#4a148c;
    classDef rot fill:#e0f2f1,stroke:#00695c,stroke-width:2px,color:#00695c;
    classDef struct fill:#eceff1,stroke:#455a64,stroke-width:2px,stroke-dasharray: 5 5,color:#37474f;
    classDef sensor fill:#fff3e0,stroke:#e65100,stroke-width:2px,color:#e65100;

    %% ==========================================
    %% 1. Power Distribution (Top Level)
    %% ==========================================
    subgraph Power_System [Power Distribution]
        direction TB
        Mains(("Main Power<br/>3Ph 240V")):::power
        Breaker["Main Breaker &<br/>E-Stop"]:::power
    end
    style Power_System fill:#ffffff,stroke:#b71c1c,stroke-width:2px,stroke-dasharray: 5 5

    %% ==========================================
    %% 2. Control System (Left Side)
    %% ==========================================
    subgraph Control_System [Control Cabinet]
        direction TB
        PLC["Main PLC<br/>Controller"]:::control
        HMI["HMI Panel"]:::control
        Comm("EtherNet/IP<br/>Bus"):::control
        
        %% Actuator Drivers
        subgraph Drives [Motor Drivers]
            direction TB
            VFD_Lift["Lift Drive<br/>(3-Phase)"]:::control
            VFD_Rot["Rotation Drive<br/>(1-Phase)"]:::control
            Relay["Brake Relay"]:::control
        end
        style Drives fill:#e1f5fe,stroke:#0277bd,stroke-width:1px
    end
    style Control_System fill:#ffffff,stroke:#0277bd,stroke-width:2px

    %% ==========================================
    %% 3. Lifting Subsystem (Container for Rotation)
    %% ==========================================
    subgraph Lift_System [Lifting Subsystem]
        direction TB
        
        %% Lifting Components
        LiftMotor("Lift Motor<br/>(3-Phase)"):::lift
        JackScrew("Jack Screw<br/>Assembly"):::lift
        
        %% Structure
        subgraph Structure [Structural Frame]
            direction TB
            Guides("Linear Guides"):::struct
            Bracing("Support Bracing"):::struct
            Shelf("Moving Shelf / Platform"):::struct
        end
        style Structure fill:#f5f5f5,stroke:#607d8b,stroke-width:1px

        %% NESTED ROTATION SUBSYSTEM (Physically on the Shelf)
        subgraph Rot_System [Rotation Subsystem (On Shelf)]
            direction TB
            RotMotor("Rot Motor<br/>(1-Phase)"):::rot
            Gearbox("Gearbox<br/>1517:1"):::rot
            Drivetrain("Coupling &<br/>Bearings"):::rot
            Shaft("Main Shaft"):::rot
            Tabletop("Tabletop Load"):::rot
            Brake("Failsafe Brake"):::rot
        end
        style Rot_System fill:#ffffff,stroke:#00695c,stroke-width:2px
        
    end
    style Lift_System fill:#ffffff,stroke:#4a148c,stroke-width:2px


    %% ==========================================
    %% 4. Feedback Sensors
    %% ==========================================
    subgraph Sensors_System [Sensors]
        LimitSw("Limit Switches"):::sensor
        Encoder("Rotary Encoders"):::sensor
    end
    style Sensors_System fill:#ffffff,stroke:#ef6c00,stroke-width:2px


    %% ==========================================
    %% CONNECTIONS
    %% ==========================================

    %% Power Flow
    Mains ==O Breaker
    Breaker ==> PLC
    Breaker ==> VFD_Lift
    Breaker ==> VFD_Rot

    %% Control Flow
    HMI <--> Comm
    Comm <--> PLC
    PLC --> Drives
    
    %% Drive Connections
    VFD_Lift ==> LiftMotor
    VFD_Rot ==> RotMotor
    Relay -.-> Brake

    %% Lifting Mechanics
    LiftMotor ==> JackScrew
    JackScrew ==> Shelf
    Guides -.-> Shelf
    Bracing -.-> Guides

    %% Support Relationship
    Shelf == "Holds" ==> RotMotor

    %% Rotation Mechanics
    RotMotor ==> Gearbox
    Gearbox ==> Drivetrain
    Drivetrain ==> Shaft
    Brake -.-> Shaft
    Shaft ==> Tabletop

    %% Feedback
    LimitSw -- "Safety" --> PLC
    Encoder -- "Feedback" --> PLC
```

---

## Detailed Subsystem Breakdown

### 1. Rotation Subsystem Diagram

```mermaid
graph LR
    linkStyle default interpolate stepAfter stroke:#555,stroke-width:2px;

    classDef rot fill:#e0f2f1,stroke:#00695c,stroke-width:2px,color:#00695c;
    classDef mech fill:#fff3e0,stroke:#e65100,stroke-width:2px,color:#e65100;
    classDef in fill:#e1f5fe,stroke:#01579b,stroke-width:2px,color:#01579b;

    Input("VFD Output<br/>(1-Phase)"):::in
    Motor("Rotation Motor<br/>(1-Phase)"):::rot
    Gearbox("Gearbox<br/>1517:1"):::mech
    
    subgraph Assm [Shaft Assembly]
        direction TB
        Coupling("Coupling"):::mech
        Bearings("Bearings"):::mech
        Shaft("Drive Shaft"):::mech
    end
    style Assm fill:#fff8e1,stroke:#ff8f00,stroke-width:1px
    
    Brake("Failsafe Brake"):::rot
    Tabletop("Tabletop Load"):::mech

    Input ==> Motor
    Motor ==> Gearbox
    Gearbox ==> Coupling
    Coupling ==> Shaft
    Bearings -.-> Shaft
    Brake -.->|"Hold"| Shaft
    Shaft ==> Tabletop
```

### 2. Lifting Subsystem Diagram

```mermaid
graph TD
    linkStyle default interpolate stepAfter stroke:#555,stroke-width:2px;

    classDef lift fill:#f3e5f5,stroke:#4a148c,stroke-width:2px,color:#4a148c;
    classDef struct fill:#eceff1,stroke:#455a64,stroke-width:2px,color:#37474f;
    classDef in fill:#e1f5fe,stroke:#01579b,stroke-width:2px,color:#01579b;

    Input("VFD Output<br/>(3-Phase)"):::in
    
    subgraph Actuation [Actuation]
        direction TB
        Motor("Lift Motor<br/>3-Phase"):::lift
        Jack("Jack Screw Assembly"):::struct
    end
    style Actuation fill:#ffffff,stroke:#4a148c,stroke-width:1px
    
    subgraph Structure [Structural Support]
        direction TB
        Guides("Linear Guides"):::struct
        Bracing("Support Bracing"):::struct
        Shelf("Equipment Shelf"):::struct
    end
    style Structure fill:#ffffff,stroke:#607d8b,stroke-width:1px
    
    Rotation("Rotation Subsystem"):::lift

    Input ==> Motor
    Motor ==> Jack
    Jack ==> Shelf
    
    Guides -.-> Shelf
    Bracing -.-> Guides
    
    Shelf ==> Rotation
```

### 3. Control Logic Flow

```mermaid
graph TD
    linkStyle default interpolate stepAfter stroke:#555,stroke-width:2px;

    classDef logic fill:#e1f5fe,stroke:#01579b,stroke-width:2px,color:#01579b;
    classDef action fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px,color:#1b5e20;
    classDef err fill:#ffebee,stroke:#b71c1c,stroke-width:2px,color:#b71c1c;

    Start((Start)):::logic
    Safety{"Safety Check?"}:::logic
    
    Start --> Safety
    Safety -- "NO" --> Halt[HALT / ERROR]:::err
    Safety -- "YES" --> Ready[System Ready]:::logic
    
    Ready --> In[/HMI Input/]:::logic
    In --> Calc[Calculate]:::logic
    
    Calc --> Drive[VFD Command]:::action
    Drive --> Move[Motion]:::action
    
    Move --> Check{"Target Reached?"}:::logic
    Check -- "NO" --> Feedback[/Read Encoders/]:::logic
    Feedback --> Calc
    
    Check -- "YES" --> Brake[Engage Brakes]:::action
    Brake --> Done((Done)):::logic
```
