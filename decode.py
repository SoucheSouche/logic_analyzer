#!/usr/bin/env python3
"""Decode TV remote protocol from raw pulse data"""

import sys

BIT_US = 102
BIT_US_HALF = 51
FRAME_GAP_US = 6000
FRAME_BITS = 48

def decode_frame(data_lines):
    """Decode one frame from pulse data and detect consecutive same-level entries.
    
    CRITICAL: The C code logs (new_level, old_duration), meaning:
    - Line i has: level[i], duration[i]
    - But duration[i] is the duration of level[i-1], not level[i]
    
    So we must pair level[i-1] with duration[i] for correct decoding.
    """
    frame = [0] * 6  # 6 bytes
    bit_counter = 0
    has_consecutive_same_level = False
    last_level = None
    prev_level = None
    
    for line in data_lines:
        parts = line.strip().split(',')
        if len(parts) != 2:
            continue
            
        level = int(parts[0].strip())
        duration = int(parts[1].strip())
        
        # Detect consecutive same-level entries (potential error)
        if last_level is not None and level == last_level:
            has_consecutive_same_level = True
        last_level = level
        
        # Skip frame gaps
        if duration >= FRAME_GAP_US:
            prev_level = level
            continue
        
        # Skip first line - no previous level to associate with this duration
        if prev_level is None:
            prev_level = level
            continue
            
        # Use PREVIOUS level with CURRENT duration
        actual_level = prev_level
        actual_duration = duration
        prev_level = level
        
        # Calculate number of bits (with rounding)
        nb_bits = (actual_duration + BIT_US_HALF) // BIT_US
        
        # Pack bits into frame
        for _ in range(nb_bits):
            if bit_counter >= FRAME_BITS:
                break
                
            byte_index = bit_counter >> 3  # Divide by 8
            bit_index = bit_counter & 7     # Modulo 8
            
            if actual_level:
                frame[byte_index] |= (1 << bit_index)
                
            bit_counter += 1
    
    return frame, bit_counter, has_consecutive_same_level

def main():
    with open('/home/guillaume/esp/esp-idf/examples/logic_analyzer/output.txt', 'r') as f:
        lines = f.readlines()
    
    # Find frame boundaries (gaps >= 6000µs)
    frames = []
    current_frame_lines = []
    
    for line in lines:
        parts = line.strip().split(',')
        if len(parts) != 2:
            continue
            
        level = int(parts[0].strip())
        duration = int(parts[1].strip())
        
        if duration >= FRAME_GAP_US:
            if current_frame_lines:
                frame, bits, has_error = decode_frame(current_frame_lines)
                if bits > 0:
                    frames.append((frame, bits, has_error))
                current_frame_lines = []
        else:
            current_frame_lines.append(line)
    
    # Don't forget the last frame
    if current_frame_lines:
        frame, bits, has_error = decode_frame(current_frame_lines)
        if bits > 0:
            frames.append((frame, bits, has_error))
    
    # Print only complete 48-bit frames WITHOUT errors
    print(f"Found {len(frames)} frames\n")
    
    from collections import Counter
    frame_counts = Counter()
    complete_frame_count = 0
    clean_frame_count = 0
    frame_number = 0
    
    for frame, bits, has_error in frames:
        frame_number += 1
        if bits == FRAME_BITS and not has_error:
            clean_frame_count += 1
            complete_frame_count += 1
            frame_hex = ' '.join(f'{b:02X}' for b in frame)
            print(f"Frame {frame_number}: {bits} bits -> {frame_hex}")
            frame_counts[frame_hex] += 1
        elif bits == FRAME_BITS:
            complete_frame_count += 1
    
    # Show statistics
    print(f"\n{'='*60}")
    print(f"Frame Statistics (48-bit frames only, errors removed):")
    print(f"{'='*60}")
    print(f"Total frames: {len(frames)}")
    print(f"Complete 48-bit frames: {complete_frame_count}")
    print(f"Clean frames (no consecutive same-level): {clean_frame_count}")
    print(f"Unique frame patterns (clean only):")
    for frame_hex, count in frame_counts.most_common():
        print(f"  {count:3}x  {frame_hex}")
    
    if frame_counts:
        most_common_frame, count = frame_counts.most_common(1)[0]
        print(f"\n✓ Most frequent frame ({count}/{complete_frame_count} occurrences):")
        print(f"  {most_common_frame}")

if __name__ == '__main__':
    main()
