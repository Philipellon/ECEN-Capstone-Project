import React, { createContext, useState, useContext } from 'react';

type WheelchairContextType = {
  lastCommandTime: Date | null;
  setLastCommandTime: (time: Date) => void;
};

const WheelchairContext = createContext<WheelchairContextType | undefined>(undefined);

export const WheelchairProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [lastCommandTime, setLastCommandTime] = useState<Date | null>(null);

  return (
    <WheelchairContext.Provider value={{ lastCommandTime, setLastCommandTime }}>
      {children}
    </WheelchairContext.Provider>
  );
};

export const useWheelchair = () => {
  const context = useContext(WheelchairContext);
  if (!context) throw new Error("useWheelchair must be used inside WheelchairProvider");
  return context;
};
