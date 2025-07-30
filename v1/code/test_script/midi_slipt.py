import sys
import mido
from mido import MidiFile, MidiTrack, Message, MetaMessage

def split_midi_track(input_file, output_file, track_index):
    """分割指定轨道的音符到两个新轨道，正确处理时间信息"""
    try:
        # 加载输入文件
        mid = MidiFile(input_file)

        # 获取目标轨道
        if track_index >= len(mid.tracks):
            raise ValueError(f"轨道索引 {track_index} 超出范围（文件只有 {len(mid.tracks)} 个轨道）")
        orig_track = mid.tracks[track_index]

        # 验证轨道是否有音符
        has_notes = any(e.type == 'note_on' and e.velocity > 0 for e in orig_track)
        if not has_notes:
            print(f"警告：轨道 {track_index} 没有音符事件！可能选择了元数据轨道")
            return

        print(f"处理轨道 {track_index}, 总事件数: {len(orig_track)}")
        print(f"音符事件数: {sum(1 for e in orig_track if e.type == 'note_on' and e.velocity > 0)}")

        # 创建两个新轨道
        track_odd = MidiTrack()  # 奇数位置的音符
        track_even = MidiTrack()  # 偶数位置的音符

        # 初始化时间计数器
        abs_time_odd = 0
        abs_time_even = 0
        abs_time_orig = 0

        # 记录每个音符的开始时间（用于结束事件）
        note_start_times = {}
        note_assignment = {}  # 记录音符被分配到哪个轨道

        # 计数器用于交替分配音符
        note_counter = 0

        # 处理所有事件
        for event in orig_track:
            abs_time_orig += event.time

            # 处理音符开始事件
            if event.type == 'note_on' and event.velocity > 0:
                note_counter += 1
                note_id = (event.channel, event.note)

                # 记录音符开始时间和分配轨道
                note_start_times[note_id] = abs_time_orig

                if note_counter % 2 == 1:
                    # 添加到奇数轨道
                    delta_time = abs_time_orig - abs_time_odd
                    new_event = Message('note_on',
                                      note=event.note,
                                      velocity=event.velocity,
                                      channel=event.channel,
                                      time=delta_time)
                    track_odd.append(new_event)
                    abs_time_odd = abs_time_orig
                    note_assignment[note_id] = 'odd'
                else:
                    # 添加到偶数轨道
                    delta_time = abs_time_orig - abs_time_even
                    new_event = Message('note_on',
                                      note=event.note,
                                      velocity=event.velocity,
                                      channel=event.channel,
                                      time=delta_time)
                    track_even.append(new_event)
                    abs_time_even = abs_time_orig
                    note_assignment[note_id] = 'even'

            # 处理音符结束事件
            elif event.type in ['note_off', 'note_on'] and event.velocity == 0:
                note_id = (event.channel, event.note)

                if note_id in note_assignment:
                    if note_assignment[note_id] == 'odd':
                        # 添加到奇数轨道
                        delta_time = abs_time_orig - abs_time_odd
                        new_event = Message('note_off',
                                          note=event.note,
                                          velocity=0,
                                          channel=event.channel,
                                          time=delta_time)
                        track_odd.append(new_event)
                        abs_time_odd = abs_time_orig
                    else:
                        # 添加到偶数轨道
                        delta_time = abs_time_orig - abs_time_even
                        new_event = Message('note_off',
                                          note=event.note,
                                          velocity=0,
                                          channel=event.channel,
                                          time=delta_time)
                        track_even.append(new_event)
                        abs_time_even = abs_time_orig

                    # 移除已处理的音符
                    del note_assignment[note_id]
                    if note_id in note_start_times:
                        del note_start_times[note_id]
                else:
                    # 没有找到开始事件，添加到两个轨道
                    delta_time_odd = abs_time_orig - abs_time_odd
                    new_event_odd = Message('note_off',
                                          note=event.note,
                                          velocity=0,
                                          channel=event.channel,
                                          time=delta_time_odd)
                    track_odd.append(new_event_odd)
                    abs_time_odd = abs_time_orig

                    delta_time_even = abs_time_orig - abs_time_even
                    new_event_even = Message('note_off',
                                          note=event.note,
                                          velocity=0,
                                          channel=event.channel,
                                          time=delta_time_even)
                    track_even.append(new_event_even)
                    abs_time_even = abs_time_orig

            # 处理非音符事件
            else:
                # 添加到两个轨道
                if event.is_meta:
                    # 元数据事件直接复制
                    event_odd = event.copy()
                    event_even = event.copy()
                else:
                    # 其他事件需要计算时间
                    delta_time_odd = abs_time_orig - abs_time_odd
                    event_odd = event.copy()
                    event_odd.time = delta_time_odd

                    delta_time_even = abs_time_orig - abs_time_even
                    event_even = event.copy()
                    event_even.time = delta_time_even

                track_odd.append(event_odd)
                track_even.append(event_even)
                abs_time_odd = abs_time_orig
                abs_time_even = abs_time_orig

        # 确保轨道有结束标记
        if not any(e.type == 'end_of_track' for e in track_odd):
            track_odd.append(MetaMessage('end_of_track', time=0))
        if not any(e.type == 'end_of_track' for e in track_even):
            track_even.append(MetaMessage('end_of_track', time=0))

        # 创建输出MIDI
        out_mid = MidiFile(ticks_per_beat=mid.ticks_per_beat)

        # 添加所有非处理轨道
        for i, track in enumerate(mid.tracks):
            if i != track_index:
                out_mid.tracks.append(track.copy())

        # 添加新分割的轨道
        out_mid.tracks.append(track_odd)
        out_mid.tracks.append(track_even)

        # 保存输出
        out_mid.save(output_file)

        # 验证结果
        notes_odd = sum(1 for e in track_odd if e.type == 'note_on' and e.velocity > 0)
        notes_even = sum(1 for e in track_even if e.type == 'note_on' and e.velocity > 0)

        print(f"\n处理完成！输出文件: {output_file}")
        print(f"原始轨道数: {len(mid.tracks)}")
        print(f"输出轨道数: {len(out_mid.tracks)}")
        print(f"新轨道1事件数: {len(track_odd)} (包含音符: {notes_odd})")
        print(f"新轨道2事件数: {len(track_even)} (包含音符: {notes_even})")
        print(f"总处理音符: {note_counter} 个")

        # 返回新轨道数据用于调试
        return track_odd, track_even

    except Exception as e:
        print(f"处理失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return None, None

def debug_midi_track(track, name):
    """调试单个轨道"""
    print(f"\n轨道 '{name}' 调试:")
    print(f"总事件数: {len(track)}")

    # 显示所有音符事件
    note_events = [e for e in track if e.type == 'note_on' and e.velocity > 0]
    print(f"音符数量: {len(note_events)}")

    # 显示前20个事件（包括时间信息）
    print("前20个事件:")
    for i, event in enumerate(track[:20]):
        print(f"{i:3}. time={event.time} | {event}")

    if len(track) > 20:
        print(f"... 更多事件 ({len(track)-20})")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python midi_splitter.py 输入文件.mid [输出文件.mid] [轨道索引]")
        print("示例: python midi_splitter.py input.mid output.mid 1")
        print("注意：轨道索引应为包含音符的轨道（通常是1或2）")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else input_file.replace('.mid', '_split.mid')
    track_index = int(sys.argv[3]) if len(sys.argv) > 3 else 1  # 默认处理轨道1

    print(f"=== 输入文件: {input_file} ===")
    print(f"输出文件: {output_file}")
    print(f"处理轨道索引: {track_index}\n")

    # 执行分割
    track_odd, track_even = split_midi_track(input_file, output_file, track_index)

    # 调试新轨道
    if track_odd:
        debug_midi_track(track_odd, "奇数音符轨道")
    if track_even:
        debug_midi_track(track_even, "偶数音符轨道")
