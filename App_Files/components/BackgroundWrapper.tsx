import React from 'react';
import { ImageBackground, StyleSheet } from 'react-native';
import { useTheme } from '../context/ThemeContext';

const BackgroundWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { theme } = useTheme();

  const bgSource = theme === 'dark'
    ? require('../assets/Loginbp.jpg')
    : require('../assets/Loginbp_light.png'); // Add your light mode image here

  return (
    <ImageBackground source={bgSource} style={styles.background} resizeMode="cover">
      {children}
    </ImageBackground>
  );
};

const styles = StyleSheet.create({
  background: {
    flex: 1,
    width: '100%',
    height: '100%',
  },
});

export default BackgroundWrapper;