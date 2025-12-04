declare module 'react-native-mjpeg' {
    import { Component } from 'react';
    
    interface MjpegViewProps {
      style?: object;
      source: { uri: string };
      bufferSize?: number;
      resizeMode?: 'cover' | 'contain' | 'stretch';
    }
  
    export default class MjpegView extends Component<MjpegViewProps> {}
  }
  