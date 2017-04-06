package org.firstinspires.ftc.teamcode.annotations_test;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.reflections.Reflections;

import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Created by roboticsteam on 4/5/2017.
 */
public class RobotCore {
    private static RobotCore ourInstance = new RobotCore();
    private static Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode");

    public static RobotCore getInstance() {
        return ourInstance;
    }

    private boolean setup = false;
    private Map<String, HardwareDevice> devices;
    private Map<String, RobotModule> modules;
    private RobotAnnotations annotations;

    private RobotCore() {
        devices = new HashMap<>();
        modules = new HashMap<>();
        annotations = RobotAnnotations.getInstance();
        moduleTypes = getModuleTypes(); //get all modules
    }

    public void setup(HardwareMap hardwareMap) {
        if(!setup) {
            for(HardwareDevice dev : hardwareMap) {
                //Put all devices in a map
                devices.put(dev.getDeviceName(), dev);
            }
        }
        setup = true;
    }

    //TODO possibly handle named constructors? Or nah?
    public RobotModule[] resolveDependencies(String... modTypes) {
        RobotModule[] resolved = new RobotModule[modTypes.length];
        for(int i = 0; i < modTypes.length; i++) {
            resolved[i] = getModuleOrCreate(modTypes[i]);
        }
        return resolved;
    }

    public <T extends HardwareDevice> T getDevice(String devName) {
        if(devices.containsKey(devName)) {
            HardwareDevice rawDevice = devices.get(devName);
            try {
                return (T) rawDevice;
            } catch (ClassCastException e) {
                throw new IllegalArgumentException("Incompatible hardware type for device " + devName);
            }
        } else {
            throw new IllegalArgumentException("No device " + devName);
        }
    }

    public RobotModule getModule(String modType) {
        if(modules.containsKey(modType)) {
            return modules.get(modType);
        } else {
            throw new IllegalArgumentException("No device " + modType);
        }
    }

    public RobotModule getModuleOrCreate(String modType) {
        return getModuleOrCreate(modType, null);
    }

    public RobotModule getModuleOrCreate(String modType, Object[] args) {
        if(modules.containsKey(modType)) {
            return modules.get(modType); //TODO should we ensure this has right type?
        } else {
            return create(modType, args);
        }
    }


    private Map<String, Class<? extends RobotModule>> moduleTypes;

    public RobotModule create(String typeName) {
        RobotModule instance = createBase(typeName, null);
        modules.put(typeName, instance);
        return instance;
    }

    public RobotModule create(String modType, Object[] args) {
        RobotModule instance = createBase(modType, args);
        modules.put(modType, instance);
        return instance;
    }

    //Creates an instance of the type associated with the name
    public RobotModule createBase(String typeName, Object[] args) {
        Class<? extends RobotModule> type = getType(typeName); //throws InvalidArgument if not valid
        try {
            if(args == null) {
                //no arguments to constructor
                return type.newInstance(); //creates instance of extending type with default constructor
            } else {
                //there are args to pass
                Class<?>[] argTypes = new Class<?>[args.length];
                for(int i = 0; i < args.length; i++) {
                    argTypes[i] = args[i].getClass();
                }
                try {
                    //Call constructor created by types of arguments
                    //TODO make sure it doesn't mess up with extended types
                    //TODO possibly loop through all constructors and test if they are valid subclass args
                    return RobotModule.class.getDeclaredConstructor(argTypes).newInstance(args);
                } catch(NoSuchMethodException|SecurityException|InvocationTargetException e) {
                    throw new IllegalArgumentException("Invalid constructor for module " + typeName);
                }
            }
        } catch(IllegalAccessException|InstantiationException e) {
            throw new IllegalArgumentException("Exception thrown from module " + typeName);
        }
    }

    public Class<? extends RobotModule> getType(String typeName) {
        if(moduleTypes.containsKey(typeName)) {
            return moduleTypes.get(typeName);
        } else {
            throw new IllegalArgumentException("No module type " + typeName);
        }
    }

    private Map<String, Class<? extends RobotModule>> getModuleTypes() {
        Map<String, Class<? extends RobotModule>> modules = new TreeMap<>();

        Set<Class<? extends RobotModule>> moduleTypes = reflections.getSubTypesOf(RobotModule.class);

        //Loop over each Class which has annotation
        for (Class<? extends RobotModule> moduleType : moduleTypes) {
            //Determine if the class is a valid RobotModule
            if (moduleType.isAnnotationPresent(Module.class)) {
                modules.put(moduleType.getAnnotation(Module.class).name(), moduleType);
            }
        }
        return modules;
    }
}
