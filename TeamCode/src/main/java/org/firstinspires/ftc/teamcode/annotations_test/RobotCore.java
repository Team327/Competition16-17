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
    private static Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode"); //Note: must be before RobotCore initialization
    private static RobotCore ourInstance = new RobotCore();

    public static RobotCore getInstance() {
        return ourInstance;
    }

    private boolean setup = false;
    private Map<String, HardwareDevice> devices; //Map of hardware devices initialized
    private Map<String, RobotModule> modules; //Map of modules initialized

    private Map<String, Class<? extends RobotModule>> moduleTypes; //Types of possible modules (used in initializing modules

    private RobotCore() {
        devices = new HashMap<>();
        modules = new HashMap<>();
        moduleTypes = getModuleTypes(); //get all modules
    }

    public void setup(HardwareMap hardwareMap) {
        //TODO TODO TODO Figure out how to iterate hardwareMap
//        if(!setup) {
//            for(HardwareDevice dev : hardwareMap) {
//                //Put all devices in a map
//                devices.put(dev.getDeviceName(), dev);
//            }
//        }
        setup = true;
    }

    //TODO possibly handle named constructors? Or nah?
    public Map<String, RobotModule> resolveDependencies(String... modTypes) {
        Map<String, RobotModule> resolved = new HashMap<>();
        for(int i = 0; i < modTypes.length; i++) {
            resolved.put(modTypes[i], getModuleOrCreate(modTypes[i]));
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

    public <T extends RobotModule> T getModule(String modType) {
        if(modules.containsKey(modType)) {
            return (T) modules.get(modType); //This could give ClassCastException
        } else {
            throw new IllegalArgumentException("No device " + modType);
        }
    }

    public <T extends RobotModule> T getModuleOrCreate(String modType) {
        return (T) getModuleOrCreate(modType, null); //This could give ClassCastException
    }

    public <T extends RobotModule> T getModuleOrCreate(String modType, Object[] args) {
        if(modules.containsKey(modType)) {
            return (T) modules.get(modType); //This could give ClassCastException //TODO should we ensure this has right type?
        } else {
            return (T) create(modType, args); //This could give ClassCastException
        }
    }

    public <T extends RobotModule> T create(String typeName) {
        RobotModule instance = createBase(typeName, null);
        modules.put(typeName, instance);
        return (T) instance; //This could give ClassCastException
    }

    public <T extends RobotModule> T create(String modType, Object[] args) {
        RobotModule instance = createBase(modType, args);
        modules.put(modType, instance);
        return (T) instance; //This could give ClassCastException
    }

    //Creates an instance of the type associated with the name
    public <T extends RobotModule> T createBase(String typeName, Object[] args) {
        Class<? extends RobotModule> type = getType(typeName); //throws InvalidArgument if not valid
        try {
            if(args == null) {
                //no arguments to constructor
                return (T) type.newInstance(); //This could give ClassCastException //creates instance of extending type with default constructor
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
                    return (T) RobotModule.class.getDeclaredConstructor(argTypes).newInstance(args); //This could give ClassCastException
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
